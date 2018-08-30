from StringIO import StringIO


def to_cpp(msg):
    """ Serialize ROS messages to string """
    buf = StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def from_cpp(serial_msg, cls):
    """ Deserialize strings to ROS messages """
    msg = cls()
    return msg.deserialize(serial_msg)
