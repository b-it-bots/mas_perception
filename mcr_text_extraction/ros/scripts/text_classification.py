#!/usr/bin/env python

import roslib; roslib.load_manifest('mcr_text_extraction')
import rospy
from std_msgs.msg import String, Float32
from stringMatching import *
from mcr_text_extraction.srv._GetExtractedText import *
from mcr_text_extraction.srv._GetRawText import *


matcher= TextClassifier(dictionary)
extracted_text = GetExtractedTextResponse()

rospy.wait_for_service('get_text')
getTextCallback= rospy.ServiceProxy('get_text', GetRawText)

def service_handler(req):
    global extracted_text
    ocrResult= str(getTextCallback().text)
    ocrResult.replace(' ', '').upper()
    label, confidence= matcher.classify(ocrResult)
    pub.publish(label)    
    extracted_text.text = String(ocrResult)
    extracted_text.label = String(label)
    extracted_text.confidence= Float32(confidence)

    return extracted_text


def listener():
    global pub
    rospy.init_node('TIEClassification', anonymous=True)
    pub = rospy.Publisher('tie_class', String)
    service = rospy.Service('get_classified_text', GetExtractedText, service_handler)
    rospy.spin()

#if __name__ == '__main__':
listener()
