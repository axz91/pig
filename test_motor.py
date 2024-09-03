from Stepper import Stepper
#from motor import Stepper

testStepper = Stepper([13,15,11 ,40,37,38])#140

#pull,dir,ena, end,rest,mag
#testStepper.step(40000, "right", 1, docking = False)
#testStepper.step(30000, "left", 50, docking = False)
testStepper.step(3000000, "right", 5, docking = True)
