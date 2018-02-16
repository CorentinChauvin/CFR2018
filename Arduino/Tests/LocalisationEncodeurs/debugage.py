# -*- coding: utf-8 -*-
"""
    Scirpt pour tester diverses fontions


Created on Fri Jun  9 15:06:00 2017

@author: corentin
"""


def updatePosition(deltaGauche, deltaDroite):
    a = 100.0
    
    
    # Rotation pure
    if (deltaGauche*deltaDroite <= 0):
        avancee = 0
        rotation = (deltaDroite-deltaGauche)  / a
    
    # Avancée
    else:
        signeAvancee =  -1 + 2*(deltaDroite > 0)
        signeRotation = -1 + 2*(abs(deltaDroite)-abs(deltaGauche) > 0)
        
        deltaDroite = abs(deltaDroite)
        deltaGauche = abs(deltaGauche)
        
        avancee = signeAvancee * min(deltaDroite, deltaGauche)
        rotation = signeRotation * (max(deltaDroite, deltaGauche)-abs(avancee)) / a
        
    print(avancee)
    print(rotation)
