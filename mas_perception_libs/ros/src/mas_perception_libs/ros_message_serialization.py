from StringIO import StringIO


def to_cpp(msg):
    """
    Serialize ROS messages to string
    :param msg: ROS message to be serialized
    :rtype: str
    """
    buf = StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def from_cpp(serial_msg, cls):
    """
    Deserialize strings to ROS messages
    :param serial_msg: serialized ROS message
    :type serial_msg: str
    :param cls: ROS message class
    :return: deserialized ROS message
    """
    msg = cls()
    return msg.deserialize(serial_msg)
