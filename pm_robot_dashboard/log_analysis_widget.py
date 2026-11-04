"""Reusable PyQt6 RSAP log assessment widget. The supplied ROS node must be spun."""

import csv
import json
from pathlib import Path
import threading

from PyQt6 import QtWidgets as Q
from PyQt6.QtCore import Qt, QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

from pm_robot_dashboard.log_analysis import (
    action_group_labels, assess, make_entry, parse_file_data,
)


class RsapLogAnalysisWidget(Q.QWidget):
    """Analyze saved runs or the latest RsapLogs snapshot with independent selections."""

    def __init__(self, node=None, parent=None):
        super().__init__(parent)
        self.node = node
        self.entries = [[], []]
        self.selected = [set(), set()]
        self.file_paths = set()
        self._export_directory = None
        self._subscription = None
        self._pending = None
        self._generation = 0
        self._lock = threading.Lock()
        self._topic_source = ''
        self.setAcceptDrops(True)
        self._build_ui()
        self._render_timer = QTimer(self)
        self._render_timer.setSingleShot(True)
        self._render_timer.setInterval(60)
        self._render_timer.timeout.connect(self.render_analysis)
        self._topic_timer = QTimer(self)
        self._topic_timer.setInterval(150)
        self._topic_timer.timeout.connect(self._consume_snapshot)
        self._topic_timer.start()
        self.refresh_topics()
        self.rebuild_entries()

    @property
    def mode(self):
        return self.source_tabs.currentIndex()

    def _button(self, layout, text, callback):
        button = Q.QPushButton(text)
        button.clicked.connect(callback)
        layout.addWidget(button)
        return button

    def _build_ui(self):
        layout = Q.QVBoxLayout(self)
        self.source_tabs = Q.QTabWidget()
        files = Q.QWidget()
        file_layout = Q.QVBoxLayout(files)
        row = Q.QHBoxLayout()
        self._button(row, 'Add files…', self.choose_files)
        self._button(row, 'Add folder…', self.choose_folder)
        self._button(row, 'Clear files', self.clear_files)
        row.addStretch()
        file_layout.addLayout(row)
        self.file_status = Q.QLabel('Drop RSAP JSON log files or folders anywhere in this widget.')
        self.file_status.setWordWrap(True)
        file_layout.addWidget(self.file_status)
        self.source_tabs.addTab(files, 'Log files')

        live = Q.QWidget()
        live_layout = Q.QVBoxLayout(live)
        row = Q.QHBoxLayout()
        self.topic_combo = Q.QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.setMinimumWidth(240)
        row.addWidget(self.topic_combo, 1)
        self._button(row, 'Find topics', self.refresh_topics)
        self.connect_button = self._button(row, 'Connect', self.connect_topic)
        self.disconnect_button = self._button(row, 'Disconnect', self.disconnect_topic)
        self.disconnect_button.setEnabled(False)
        self.durability = Q.QComboBox()
        self.durability.addItems(['Retained snapshot', 'Volatile'])
        self.durability.setToolTip('Retained snapshot receives the last RSAP message on connection.')
        row.addWidget(self.durability)
        live_layout.addLayout(row)
        self.topic_status = Q.QLabel('Latest snapshot only; repeated messages do not accumulate runs.')
        self.topic_status.setWordWrap(True)
        live_layout.addWidget(self.topic_status)
        if self.node is None:
            self.connect_button.setEnabled(False)
            self.topic_status.setText('File mode is available. Supply a spinning ROS node for live logs.')
        self.source_tabs.addTab(live, 'Live topic')
        self.source_tabs.currentChanged.connect(self.rebuild_entries)
        layout.addWidget(self.source_tabs)

        splitter = Q.QSplitter(Qt.Orientation.Horizontal)
        left = Q.QWidget()
        left_layout = Q.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.search = Q.QLineEdit()
        self.search.setPlaceholderText('Find an action or source (display filter only)…')
        self.search.textChanged.connect(self.filter_tree)
        left_layout.addWidget(self.search)
        for buttons in (
            [('Select all', 'all'), ('Select none', 'none'), ('Invert', 'invert')],
            [('Deselect failed', 'failed'), ('Only successful', 'successful')],
        ):
            row = Q.QHBoxLayout()
            for text, operation in buttons:
                self._button(row, text, lambda checked=False, op=operation: self.select_entries(op))
            left_layout.addLayout(row)
        self.tree = Q.QTreeWidget()
        self.tree.setHeaderLabels(['Include / action', 'Duration [s]', 'Result'])
        self.tree.setAlternatingRowColors(True)
        self.tree.setColumnWidth(0, 290)
        self.tree.itemChanged.connect(self._entry_changed)
        self.tree.currentItemChanged.connect(self.show_details)
        self.tree.itemDoubleClicked.connect(lambda *_args: self.output_tabs.setCurrentIndex(2))
        left_layout.addWidget(self.tree)
        note = Q.QLabel('Check a source to toggle its entries. Invalid durations are excluded. '
                        'Selection buttons affect all entries, including hidden search results.')
        note.setWordWrap(True)
        left_layout.addWidget(note)
        splitter.addWidget(left)

        right = Q.QWidget()
        right_layout = Q.QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.summary = Q.QLabel()
        self.summary.setWordWrap(True)
        right_layout.addWidget(self.summary)
        settings = Q.QGroupBox('Chart options')
        grid = Q.QGridLayout(settings)
        self.chart_type = Q.QComboBox()
        self.chart_type.addItems(['Mean duration per action', 'Duration distribution',
                                  'Total duration per source', 'Individual durations'])
        self.sort_order = Q.QComboBox()
        self.sort_order.addItems(['Longest first', 'Name', 'Sequence / input order'])
        self.grouping = Q.QComboBox()
        self.grouping.addItems(['Action ID + name', 'Action ID + index + name'])
        self.grouping.setToolTip('Only entries with the same action UUID are combined. '
                                 'Entries without an ID remain separate.')
        self.units = Q.QComboBox()
        self.units.addItems(['Seconds', 'Minutes'])
        self.minimum = Q.QDoubleSpinBox()
        self.minimum.setRange(0, 1e9)
        self.minimum.setSuffix(' s')
        self.minimum.setToolTip('Hide chart groups with a mean below this duration. Statistics remain unchanged.')
        self.axis_max = Q.QDoubleSpinBox()
        self.axis_max.setRange(0, 1e9)
        self.axis_max.setSpecialValueText('Auto')
        self.axis_max.setToolTip('Duration axis maximum in the selected display unit; zero means automatic.')
        self.top_n = Q.QSpinBox()
        self.top_n.setRange(0, 10000)
        self.top_n.setSpecialValueText('All')
        self.top_n.setValue(30)
        self.top_n.setToolTip('Limit chart rows only; statistics include the full selection. Zero shows all.')
        self.error_bars = Q.QCheckBox('Standard deviation')
        self.error_bars.setChecked(True)
        self.value_labels = Q.QCheckBox('Value labels')
        self.value_labels.setChecked(True)
        controls = [('Chart', self.chart_type), ('Sort', self.sort_order),
                    ('Group by', self.grouping), ('Units', self.units),
                    ('Minimum mean', self.minimum), ('Axis maximum', self.axis_max),
                    ('Maximum rows', self.top_n)]
        for number, (label, control) in enumerate(controls):
            row, column = divmod(number, 2)
            grid.addWidget(Q.QLabel(label), row, column * 2)
            grid.addWidget(control, row, column * 2 + 1)
            signal = (control.currentIndexChanged if isinstance(control, Q.QComboBox)
                      else control.valueChanged)
            signal.connect(self.schedule_render)
        grid.addWidget(self.error_bars, 3, 2)
        grid.addWidget(self.value_labels, 3, 3)
        self.error_bars.toggled.connect(self.schedule_render)
        self.value_labels.toggled.connect(self.schedule_render)
        right_layout.addWidget(settings)

        self.output_tabs = Q.QTabWidget()
        chart = Q.QWidget()
        chart_layout = Q.QVBoxLayout(chart)
        self.figure = Figure(figsize=(8, 5), constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        chart_layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        scroll = Q.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.canvas)
        chart_layout.addWidget(scroll)
        self.output_tabs.addTab(chart, 'Chart')
        stats = Q.QWidget()
        stats_layout = Q.QVBoxLayout(stats)
        self.stats_table = Q.QTableWidget(0, 8)
        self.stats_table.setHorizontalHeaderLabels(
            ['Action', 'Count', 'Mean [s]', 'Std [s]', 'Min [s]', 'Max [s]',
             'Total [s]', 'Total [min]'])
        self.stats_table.setEditTriggers(Q.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.stats_table.horizontalHeader().setSectionResizeMode(Q.QHeaderView.ResizeMode.ResizeToContents)
        stats_layout.addWidget(self.stats_table, 2)
        self.stats_total = Q.QLabel()
        self.stats_total.setWordWrap(True)
        stats_layout.addWidget(self.stats_total)
        self.source_totals_table = Q.QTableWidget(0, 3)
        self.source_totals_table.setHorizontalHeaderLabels(['Source', 'Total [s]', 'Total [min]'])
        self.source_totals_table.setEditTriggers(Q.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.source_totals_table.horizontalHeader().setSectionResizeMode(
            Q.QHeaderView.ResizeMode.ResizeToContents)
        stats_layout.addWidget(self.source_totals_table, 1)
        self._button(stats_layout, 'Export selected statistics…', self.export_statistics)
        self.output_tabs.addTab(stats, 'Statistics')
        self.details = Q.QPlainTextEdit()
        self.details.setReadOnly(True)
        self.output_tabs.addTab(self.details, 'Selected entry JSON')
        right_layout.addWidget(self.output_tabs, 1)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([430, 750])
        layout.addWidget(splitter, 1)

    def schedule_render(self, *_args):
        self._render_timer.start()

    def choose_files(self):
        paths, _ = Q.QFileDialog.getOpenFileNames(self, 'Open RSAP logs', '', 'JSON logs (*.json)')
        self.load_paths(paths)

    def choose_folder(self):
        path = Q.QFileDialog.getExistingDirectory(self, 'Open folder of RSAP logs')
        if path:
            self.load_paths([path])

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls() and any(url.isLocalFile() for url in event.mimeData().urls()):
            event.acceptProposedAction()

    def dropEvent(self, event):
        self.source_tabs.setCurrentIndex(0)
        self.load_paths([url.toLocalFile() for url in event.mimeData().urls() if url.isLocalFile()])
        event.acceptProposedAction()

    def load_paths(self, paths):
        """Add files once; folders include their immediate JSON files."""
        candidates, errors = set(), []
        for value in paths:
            path = Path(value).expanduser()
            try:
                candidates.update(path.glob('*.json') if path.is_dir() else [path])
            except OSError as exc:
                errors.append(f'{path}: {exc}')
        added = 0
        seen_executions = {entry.log.get('execution_id') for entry in self.entries[0]
                           if entry.log.get('execution_id')}
        for path in sorted(candidates):
            source = str(path.resolve())
            if source in self.file_paths:
                continue
            try:
                with path.open(encoding='utf-8') as stream:
                    entries, warnings = parse_file_data(json.load(stream), source)
                unique_entries = []
                for entry in entries:
                    execution_id = entry.log.get('execution_id')
                    if execution_id and execution_id in seen_executions:
                        continue
                    if execution_id:
                        seen_executions.add(execution_id)
                    unique_entries.append(entry)
                entries = unique_entries
                self.entries[0].extend(entries)
                self.selected[0].update(entry.key for entry in entries if entry.duration is not None)
                self.file_paths.add(source)
                self._export_directory = Path(source).parent
                errors.extend(f'{path.name}: {warning}' for warning in warnings)
                added += 1
            except (OSError, ValueError) as exc:
                errors.append(f'{path.name}: {exc}')
        self.file_status.setText(
            f'{len(self.file_paths)} files loaded · {added} added · {len(errors)} read/format issues. '
            'Drop more JSON files or folders to compare runs.')
        self.file_status.setToolTip('\n'.join(errors))
        if self.mode == 0:
            self.rebuild_entries()
        if errors:
            Q.QMessageBox.warning(self, 'Some log data could not be loaded', '\n'.join(errors[:20]))

    def clear_files(self):
        self.entries[0].clear()
        self.selected[0].clear()
        self.file_paths.clear()
        self._export_directory = None
        self.file_status.setText('Drop RSAP JSON log files or folders anywhere in this widget.')
        if self.mode == 0:
            self.rebuild_entries()

    def refresh_topics(self):
        previous = self.topic_combo.currentText()
        topics = []
        if self.node is not None:
            try:
                topics = [name for name, types in self.node.get_topic_names_and_types()
                          if 'rsap_interfaces/msg/RsapLogs' in types]
            except Exception as exc:
                self.topic_status.setText(f'Topic discovery failed: {exc}')
        self.topic_combo.clear()
        self.topic_combo.addItems(sorted(topics))
        self.topic_combo.setCurrentText(previous or (topics[0] if topics else
                                        '/ros_sequential_action_programmer/rsap_logs'))

    def connect_topic(self):
        if self.node is None:
            return
        self.disconnect_topic()
        topic = self.topic_combo.currentText().strip()
        try:
            from rclpy.qos import QoSProfile, DurabilityPolicy
            from rsap_interfaces.msg import RsapLogs
            qos = QoSProfile(depth=1, durability=(
                DurabilityPolicy.TRANSIENT_LOCAL if self.durability.currentIndex() == 0
                else DurabilityPolicy.VOLATILE))
            generation = self._generation

            def receive(message):
                # ROS executor threads only replace one pending snapshot. Qt owns the UI.
                with self._lock:
                    if generation == self._generation:
                        self._pending = message

            self._subscription = self.node.create_subscription(RsapLogs, topic, receive, qos)
            self._topic_source = topic
            self.entries[1] = []
            self.selected[1] = set()
            self.topic_status.setText(f'Waiting for {topic} · latest snapshot only')
            self.disconnect_button.setEnabled(True)
            self.connect_button.setEnabled(False)
            self.topic_combo.setEnabled(False)
            self.durability.setEnabled(False)
            if self.mode == 1:
                self.rebuild_entries()
        except Exception as exc:
            self.topic_status.setText(f'Could not subscribe: {exc}')

    def disconnect_topic(self):
        with self._lock:
            self._generation += 1
            self._pending = None
        if self._subscription is not None:
            self.node.destroy_subscription(self._subscription)
            self._subscription = None
        self.disconnect_button.setEnabled(False)
        self.connect_button.setEnabled(self.node is not None)
        self.topic_combo.setEnabled(True)
        self.durability.setEnabled(True)
        self.topic_status.setText('Disconnected · last received snapshot remains available')

    def _consume_snapshot(self):
        with self._lock:
            message, self._pending = self._pending, None
        if message is None:
            return
        source = self._topic_source
        entries, errors = [], []
        for position, record in enumerate(message.logs):
            try:
                log = json.loads(record.log_json)
                if not isinstance(log, dict):
                    raise ValueError('log_json must contain an object')
                identity = log.get("execution_id") or (record.action_uuid, record.action_index, position)
                entries.append(make_entry(source, identity, record.action_index,
                                          record.action_name, log,
                                          record.action_uuid, record.success))
            except (ValueError, TypeError) as exc:
                errors.append(f'Entry {position}: {exc}')
        old_keys = {entry.key for entry in self.entries[1]}
        self.selected[1] = {entry.key for entry in entries if entry.duration is not None
                            and (entry.key in self.selected[1] or entry.key not in old_keys)}
        self.entries[1] = entries
        self.topic_status.setText(
            f'{source} · {message.sequence_name or "Unnamed sequence"} · '
            f'{message.stamp.sec}.{message.stamp.nanosec:09d} · '
            f'{len(entries)} entries · {len(errors)} format issues · latest snapshot only')
        self.topic_status.setToolTip('\n'.join(errors))
        if self.mode == 1:
            self.rebuild_entries()

    def rebuild_entries(self, *_args):
        scroll_position = self.tree.verticalScrollBar().value()
        current = self.tree.currentItem()
        current_entry = current.data(0, Qt.ItemDataRole.UserRole) if current else None
        current_key = current_entry.key if current_entry else None
        expanded = {self.tree.topLevelItem(index).toolTip(0):
                    self.tree.topLevelItem(index).isExpanded()
                    for index in range(self.tree.topLevelItemCount())}
        self.tree.blockSignals(True)
        self.tree.clear()
        self.details.clear()
        roots = {}
        selected_item = None
        for entry in self.entries[self.mode]:
            if entry.source not in roots:
                label = Path(entry.source).name if self.mode == 0 else entry.source
                root = Q.QTreeWidgetItem(self.tree, [label])
                root.setToolTip(0, entry.source)
                root.setFlags(root.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                roots[entry.source] = root
            result = 'Success' if entry.success is True else ('Failed' if entry.success is False else 'Unknown')
            item = Q.QTreeWidgetItem(roots[entry.source], [
                f'{entry.index}: {entry.name}',
                f'{entry.duration:.3f}' if entry.duration is not None else 'Unavailable', result])
            item.setData(0, Qt.ItemDataRole.UserRole, entry)
            if entry.key == current_key:
                selected_item = item
            item.setToolTip(0, entry.issue or f'{entry.source}\nUUID: {entry.uuid}')
            if entry.duration is not None:
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(0, Qt.CheckState.Checked if entry.key in self.selected[self.mode]
                                   else Qt.CheckState.Unchecked)
            else:
                item.setForeground(1, Qt.GlobalColor.darkRed)
        for source, root in roots.items():
            root.setExpanded(expanded.get(source, True))
        self._sync_source_checks()
        if selected_item is not None:
            self.tree.setCurrentItem(selected_item)
            self.show_details(selected_item)
        self.tree.blockSignals(False)
        self.filter_tree()
        self.tree.verticalScrollBar().setValue(scroll_position)
        self.schedule_render()

    def filter_tree(self, *_args):
        query = self.search.text().casefold().strip()
        for index in range(self.tree.topLevelItemCount()):
            root = self.tree.topLevelItem(index)
            visible = False
            for number in range(root.childCount()):
                child = root.child(number)
                entry = child.data(0, Qt.ItemDataRole.UserRole)
                matches = query in f'{entry.name} {entry.source}'.casefold()
                child.setHidden(not matches)
                visible |= matches
            root.setHidden(not visible)

    def _sync_source_checks(self):
        for index in range(self.tree.topLevelItemCount()):
            root = self.tree.topLevelItem(index)
            children = [root.child(number) for number in range(root.childCount())
                        if root.child(number).data(0, Qt.ItemDataRole.UserRole).duration is not None]
            count = sum(child.checkState(0) == Qt.CheckState.Checked for child in children)
            state = (Qt.CheckState.Checked if children and count == len(children)
                     else Qt.CheckState.PartiallyChecked if count else Qt.CheckState.Unchecked)
            root.setCheckState(0, state)

    def _entry_changed(self, changed, _column):
        self.tree.blockSignals(True)
        if changed.parent() is None:
            for number in range(changed.childCount()):
                child = changed.child(number)
                if child.data(0, Qt.ItemDataRole.UserRole).duration is not None:
                    child.setCheckState(0, changed.checkState(0))
        selected = set()
        for index in range(self.tree.topLevelItemCount()):
            root = self.tree.topLevelItem(index)
            for number in range(root.childCount()):
                item = root.child(number)
                entry = item.data(0, Qt.ItemDataRole.UserRole)
                if entry.duration is not None and item.checkState(0) == Qt.CheckState.Checked:
                    selected.add(entry.key)
        self.selected[self.mode] = selected
        self._sync_source_checks()
        self.tree.blockSignals(False)
        self.schedule_render()

    def select_entries(self, operation):
        entries = [entry for entry in self.entries[self.mode] if entry.duration is not None]
        selected = self.selected[self.mode]
        if operation == 'all':
            selected = {entry.key for entry in entries}
        elif operation == 'none':
            selected = set()
        elif operation == 'invert':
            selected = {entry.key for entry in entries if entry.key not in selected}
        elif operation == 'failed':
            selected = {entry.key for entry in entries
                        if entry.key in selected and entry.success is not False}
        elif operation == 'successful':
            selected = {entry.key for entry in entries if entry.success is True}
        self.selected[self.mode] = selected
        self.rebuild_entries()

    def show_details(self, item, _previous=None):
        entry = item.data(0, Qt.ItemDataRole.UserRole) if item is not None else None
        self.details.setPlainText(json.dumps(entry.log, indent=2, ensure_ascii=False) if entry else '')

    def assessment(self):
        return assess(self.entries[self.mode], self.selected[self.mode],
                      group_by_index=self.grouping.currentIndex() == 1)

    def render_analysis(self):
        groups, stats, totals = self.assessment()
        entries = [entry for entry in self.entries[self.mode]
                   if entry.key in self.selected[self.mode] and entry.duration is not None]
        invalid = sum(entry.duration is None for entry in self.entries[self.mode])
        total_duration = sum(totals.values())
        self.summary.setText(
            f'{len(entries)} / {len(self.entries[self.mode])} entries included · '
            f'{len(groups)} actions · total {total_duration:.3f} s ({total_duration / 60:.3f} min) · '
            f'{sum(entry.success is False for entry in entries)} failed · {invalid} invalid durations. '
            'Totals sum selected action durations; they exclude gaps between actions.')
        self.stats_total.setText(
            f'Total selected duration: {total_duration:.3f} s ({total_duration / 60:.3f} min)')
        self.source_totals_table.setRowCount(len(totals))
        for row, (source, duration) in enumerate(totals.items()):
            source_label = Path(source).name if self.mode == 0 else source
            for column, value in enumerate([source_label, f'{duration:.3f}', f'{duration / 60:.3f}']):
                item = Q.QTableWidgetItem(value)
                item.setToolTip(source)
                self.source_totals_table.setItem(row, column, item)
        labels = list(stats)
        if self.sort_order.currentIndex() == 0:
            labels.sort(key=lambda label: stats[label]['mean'], reverse=True)
        elif self.sort_order.currentIndex() == 1:
            labels.sort(key=str.casefold)
        self.stats_table.setRowCount(len(labels))
        for row, label in enumerate(labels):
            values = [label, str(stats[label]['count'])] + [
                f'{stats[label][key]:.3f}' for key in ('mean', 'std', 'min', 'max', 'total')]
            values.append(f'{stats[label]["total"] / 60:.3f}')
            for column, value in enumerate(values):
                self.stats_table.setItem(row, column, Q.QTableWidgetItem(value))
        self.figure.clear()
        axes = self.figure.add_subplot(111)
        divisor = 60 if self.units.currentIndex() == 1 else 1
        unit = 'min' if divisor == 60 else 's'
        chart_type = self.chart_type.currentIndex()
        labels = [label for label in labels if stats[label]['mean'] >= self.minimum.value()]
        entry_labels = action_group_labels(
            self.entries[self.mode], self.grouping.currentIndex() == 1)
        display_labels = {}
        for entry in self.entries[self.mode]:
            name = (f'{entry.index}_{entry.name}' if self.grouping.currentIndex() == 1
                    else entry.name)
            display_labels.setdefault(entry_labels[entry.key], name)
        if chart_type in (0, 1):
            if self.top_n.value():
                labels = labels[:self.top_n.value()]
            values = [stats[label]['mean'] / divisor for label in labels]
            if labels and chart_type == 0:
                errors = [stats[label]['std'] / divisor for label in labels]
                bars = axes.barh(range(len(labels)), values, color='#337da8',
                                 xerr=errors if self.error_bars.isChecked() else None, capsize=3)
                axes.set_yticks(range(len(labels)), [self._chart_label(display_labels[label]) for label in labels])
                if self.value_labels.isChecked():
                    axes.bar_label(bars, fmt='%.2f', padding=4)
            elif labels:
                axes.boxplot([[value / divisor for value in groups[label]] for label in labels],
                             vert=False, labels=[self._chart_label(display_labels[label]) for label in labels], showmeans=True)
        else:
            if chart_type == 2:
                pairs = [(source, total / divisor) for source, total in totals.items()]
            else:
                allowed = set(labels)
                pairs = [(f'{Path(entry.source).name} · {entry.index}: {entry.name}',
                          entry.duration / divisor) for entry in entries
                         if entry_labels[entry.key] in allowed]
            if self.sort_order.currentIndex() == 0:
                pairs.sort(key=lambda pair: pair[1], reverse=True)
            elif self.sort_order.currentIndex() == 1:
                pairs.sort(key=lambda pair: pair[0].casefold())
            if self.top_n.value():
                pairs = pairs[:self.top_n.value()]
            labels, values = [pair[0] for pair in pairs], [pair[1] for pair in pairs]
            if labels:
                chart_labels = [Path(label).name for label in labels] if chart_type == 2 else labels
                bars = axes.barh(range(len(labels)), values, color='#337da8')
                axes.set_yticks(range(len(labels)), [self._chart_label(label) for label in chart_labels])
                if self.value_labels.isChecked():
                    axes.bar_label(bars, fmt='%.2f', padding=4)
        self.canvas.setMinimumHeight(max(340, len(labels) * 27 + 100))
        if not labels:
            axes.text(.5, .5, 'No entries match the selection and chart options',
                      ha='center', va='center', transform=axes.transAxes)
        axes.set_title(self.chart_type.currentText())
        axes.set_xlabel(f'Duration [{unit}]')
        axes.invert_yaxis()
        axes.grid(axis='x', alpha=.2)
        axes.set_axisbelow(True)
        axes.set_xlim(left=0)
        if self.axis_max.value() > 0:
            axes.set_xlim(right=self.axis_max.value())
        else:
            axes.margins(x=.15)
        self.canvas.draw_idle()

    @staticmethod
    def _chart_label(label):
        # Full names remain available in the entry tree and statistics table.
        return label if len(label) <= 42 else label[:39] + '…'

    def export_statistics(self):
        default_path = Path('rsap_duration_stats.json')
        if self.mode == 0 and self._export_directory is not None:
            default_path = self._export_directory / default_path
        path, _ = Q.QFileDialog.getSaveFileName(
            self, 'Export selected statistics', str(default_path),
            'JSON (*.json);;CSV (*.csv)')
        if not path:
            return
        _, stats, totals = self.assessment()
        try:
            with open(path, 'w', encoding='utf-8', newline='') as stream:
                if Path(path).suffix.lower() == '.csv':
                    writer = csv.writer(stream)
                    writer.writerow(['action', 'count', 'mean_s', 'std_s', 'min_s', 'max_s', 'total_s'])
                    for label, values in stats.items():
                        writer.writerow([label] + [values[key] for key in
                                                   ('count', 'mean', 'std', 'min', 'max', 'total')])
                    writer.writerow([])
                    writer.writerow(['source', 'total_duration_s'])
                    writer.writerows(totals.items())
                else:
                    json.dump({'per_process_stats': stats, 'total_duration_per_file': totals,
                               'duration_unit': 'seconds'}, stream, indent=2)
        except OSError as exc:
            Q.QMessageBox.warning(self, 'Export failed', str(exc))

    def shutdown(self):
        """Release the subscription before the host destroys its ROS node."""
        self._topic_timer.stop()
        self._render_timer.stop()
        self.disconnect_topic()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)
