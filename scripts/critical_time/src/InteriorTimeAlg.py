import numpy as np

def findTimeLeft(intTemp, ambTemp, time):
    table = makeTables() # initializing all the data
    intTemp = np.round(intTemp)
    ambTemp = np.round(ambTemp)
    if (intTemp % 5 == 0) and (ambTemp % 5 == 0) : # if both are increment of 5
          i = findTableIndex(intTemp, ambTemp)
          getValue(table, i)
    else:  # at least one of the values is not an increment of 5
        intTempRange = [0,0]
        ambTempRange = [0,0]
        if (intTemp % 5 == 0):  #if only internal temp is increment of 5
            intTempRange = [intTemp, intTemp]
            ambTempRange = getBelowAndAbove(ambTemp)
        elif (ambTemp % 5 == 0): # if only ambient temp is an increment of 5
            ambTempRange = [ambTemp, ambTemp]
            intTempRange = getBelowAndAbove(intTemp)
        else: # neither is an increment of 5
            intTempRange = getBelowAndAbove(intTemp) 
            ambTempRange = getBelowAndAbove(ambTemp)
        indexBelow = findTableIndex(intTempRange[0], ambTempRange[0])
        indexAbove = findTableIndex(intTempRange[1], ambTempRange[1])
        valBelow = getValue(table, indexBelow)
        valAbove = getValue(table,indexAbove)
        xp = [intTempRange[0],intTempRange[1]]
        fp = [valueBelow,valueAbove]
        currentTime = np.interp(intTemp, xp, fp)
        timeAt60 = getValue(table, findTableIndex(60, ambTemp))
    return timeAt60 - currentTime

# initializes the data
def makeTables():
    equations = np.zeros([24,10])
    for i in range(9):
        equations[:,i] = np.arange(0,24)
    # -- Put all the equations here --
    return equations

# takes internal temperature, ambient temperature, in increment of 5
# returns the index where these values are located
def findTableIndex(intTemp, ambTemp):
    ambIndex = np.where(np.arange(100,220,5) == ambTemp)
    intIndex = np.where(np.arange(20,60,5) == intTemp)
    return [intIndex, ambIndex]


# takes in a temperature and 
# returns the closest increment of 5 above and below
def getBelowAndAbove(temp):
    below = temp
    above = temp
    vals = [below,above] 
    for i in range (5):
        below = below - 1
        above = above + 1
        if (below % 5 == 0):
            vals[0] = below
        if (above % 5 == 0):
            vals[1] = above
   
    return vals
    
    
    
def getValue(table, index):
    return "value"

print(makeTables())

    