def clean_topic_name(topic: str) -> str:
    """
    If topic name starts with digit, prepend 'N'.
    """

    if topic[0].isdigit():
        topic = "N" + topic
    
    return topic