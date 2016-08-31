#Those are all characteristics of the system, and they 
#do not have anything to do with the algorithm itself,
#they are dependednt only on the system we are modeling

# The prior beliefs at X0
belXoOpen  = 0.5
belXoClosed  = 0.5

belX0 = [belXoOpen,belXoClosed]
#The sensors are noise
pOpen_xtOpen = 0.6
pClosed_xtOpen = 0.4
pOpen_xtClosed = 0.2
pClosed_xtClosed = 0.8

#Controls are also noisy
pOpen_uPush_xOpen = 1
pClosed_uPush_xOpen = 0
pOpen_uPush_xClosed = 0.8
pClosed_uPush_xClosed = 0.2

pOpen_uNothing_xOpen = 1
pClosed_uNothing_xOpen = 0
pOpen_uNothing_xClosed = 0
pClosed_uNothing_xClosed = 1


#belXt_1 will be a list [open, closed]
def bayes_filter(belXt_1,ut,zt):
	#ut==0 do nothing ut==1 push
	if ut==0:
		bel_prediction_open = pOpen_uNothing_xOpen * belXt_1[0] + pOpen_uNothing_xClosed * belXt_1[1]
		bel_prediction_closed = pClosed_uNothing_xOpen * belXt_1[0] + pClosed_uNothing_xClosed * belXt_1[1]
	if ut==1:
		bel_prediction_open = pOpen_uPush_xOpen * belXt_1[0] + pOpen_uPush_xClosed * belXt_1[1]
		bel_prediction_closed = pClosed_uPush_xOpen * belXt_1[0] + pClosed_uPush_xClosed * belXt_1[1]

	# print "Predictions:",[bel_prediction_open,bel_prediction_closed]
	#after we make the measurments
	#zt==0 means closed zt==1 means opened
	if zt==0:
		belOpen = pClosed_xtOpen * bel_prediction_open
		belClosed = pClosed_xtClosed * bel_prediction_closed
	if zt==1:
		belOpen = pOpen_xtOpen* bel_prediction_open 
		belClosed = pOpen_xtClosed* bel_prediction_closed

	n = (belOpen + belClosed) ** -1
	belOpenNormalized = n* belOpen
	belClosedNormalized = n* belClosed
	return [belOpenNormalized,belClosedNormalized]

X0 =  bayes_filter([0.5,0.5],0,1)
X1 = bayes_filter(X0,1,1)

print X0
print X1


