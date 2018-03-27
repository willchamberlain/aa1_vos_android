#!/usr/bin/env python

class AClass: 
	aClassCounter = 100

	def printCounter(self):
		print "AClass %d"%(AClass.aClassCounter)
		AClass.aClassCounter = AClass.aClassCounter+10

	def __init__(self):
		self.printCounter()


class BClass: 
	bClassCounter = 0

	def __init__(self):
		self.printCounter()

	def printCounter(self):
		print "BClass %d"%(BClass.bClassCounter)
		BClass.bClassCounter = BClass.bClassCounter+1
