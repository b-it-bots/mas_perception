#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''

Author: Jose Antonio Alvarez Ruiz
E-mail: jose.alvarez@smail.inf.h-brs.de
Date: 31/03/2011


Allows to classify objects for which some embedded text strings are known.

'''

from Levenshtein import *
from itertools import *


dictionary= {
    'POTATO PRODUCT' : ['KARTOFFEL'],
    'PUREE' : ['PUREE'],
    'SAUCE' : ['SAUCE'],
    'TOMATO PRODUCT' : ['TOMATE', 'TOMATO'],
    'ORANGE PRODUCT' : ['ORANGE'],
    'RICE PRODUCT' : ['REIS'],
    'JUICE' : ['HOHES', 'NEKTAR'],
    'SNACK' : ['SNACK'],
    'DRINK' : ['TRINK', 'ERFRISCH', 'PFAND'],
    'FOOD' : ['ESSEN', 'LECKER', 'SCHMECK'],
    'WATER' : ['WASSER', 'AQUA', 'QUELLE', 'SASKIA'],
    'MILK' : ['VOLL', 'MILCH', 'HALDBARE', 'LAKTOSE', 'MILUMIL'],
    'TOILET PRODUCT' : ['WC', 'TOILET'],
    'CLEANING PRODUCT' : ['REINIGER', 'WASCH', 'SPUL', 'PUTZ', 'DRECK', 'FLECK'],
    'COOKIES' : ['MANDEL','SPRITZ','KECKS'],
    'SPICES' : ['MAGGIE','WURZE'],
    'CHILDREN PRODUCT' : ['KINDER', 'CHILDREN'],
    'CHOCOLATE' : ['SCHOCOLADE', 'CHOCOLATE', 'KAKAO'],

    }

def mergeComposite(keys, dictionary= dictionary):
    return reduce(list.__add__, map(dictionary.__getitem__, keys))

dictionary ['POTATO PUREE']= mergeComposite(['POTATO PRODUCT', 'PUREE'])
dictionary ['ORANGE JUICE']= mergeComposite(['ORANGE PRODUCT', 'JUICE'])
dictionary ['TOMATO PUREE']= mergeComposite(['TOMATO PRODUCT', 'PUREE'])
dictionary ['TOILET CLEANING PRODUCT']= mergeComposite(['TOILET PRODUCT', 'CLEANING PRODUCT'])
dictionary ['EATABLE']= mergeComposite(['FOOD', 'DRINK',
                                        'TOMATO PRODUCT', 'POTATO PRODUCT',
                                        'PUREE', ''
                                        'MILK',
                                        'SAUCE',
                                        'SPICES',
                                        'COOKIES'])




def expandDictionaryEntries(dictionary):
    result= {}
    for k,v in dictionary.iteritems():
        # print v
        r= []
        for n in range(3):
            for i, vv in enumerate(permutations(v, n+1)):
                r.append(''.join(list(vv)))
        result[k]= r
    return result

def mergeComposite(keys, dictionary= dictionary):
    return reduce(list.__add__, map(dictionary.__getitem__, keys))

dictionary ['POTATO PUREE']= mergeComposite(['POTATO PRODUCT', 'PUREE'])
dictionary ['TOILET CLEANING PRODUCT']= mergeComposite(['TOILET PRODUCT', 'CLEANING PRODUCT'])


class TextClassifier:
    def __init__(self, dictionary):
        self.dictionary= dictionary
        self.initializeMatcher()

    def initializeMatcher(self):
        ''' Creates a dictionary that maps class labels to permutation
        strings. Each permutation string is formed by concatenating
        the strings of cue words defined for each label in the
        dictionary. For example if you have a dictionary:

        d= {"class1" : ("word1", "word2"),
        "class2" : ("word3", word4)
        }

        The corresponding permutation dictionary would be:

        p= {"class1" : ("word1word2", "word2word1"),
        "class2" : ("word3word4", "word4word3")
        }
        
        '''
        self.dictPerm= expandDictionaryEntries(self.dictionary)

    def classify(self, s):
        ''' Classifies a string as one of the classes defined in the
        dictionary. The scoring of each class is the maximum
        similarity score with s, among the permutation strings of the
        corresponding class. The class with highest score is
        considered the to correspond to s '''
        classes= self.dictPerm.keys()
        scores= map(lambda c:
                    max(map(lambda ss: ratio(s, ss), self.dictPerm[c])),
                    classes)
        confidence= max(scores)
        return (classes[scores.index(max(scores))], confidence)


# Example
if __name__ == '__main__':
    inputStrings=('K0ART987123L', '9PUR8HCW', 'REE87*(OFEL', 'SAKIA', 'VOLLMILCH','ANDT8R1IN123')
    tc= TextClassifier(dictionary)
    for s in inputStrings:
        print s, tc.classify(s)        
