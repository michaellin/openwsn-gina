def detectTaps(gyrX,time,state):

	# states:
	state = 0 ;#-> random Motion
	state = 0.5 ;#-> randomMotion to rest transition (checking for time)
	state = 1 ;#-> rest
	state = 2 ;#-> triggerUp
	state = 3 ;#-> triggerDown
	state = 2.5 ;#-> trnsUpDown
	state = 3.5 ;#-> trnsDownRest
	##############################
	#variables
	#states
	#state = 0 ;#-> random Motion
	state = 1 ;#-> rest
	#thresholds
	#magnitude
	minUprThrshld = 2800.*2000*3.14159265359/180/65535;#2500;#3000;
	maxUprThrshld = 18000.*2000*3.14159265359/180/65535;
	minLwrThrshld = -4000.*2000*3.14159265359/180/65535;#-3000;
	maxLwrThrshld = -33000.*2000*3.14159265359/180/65535;
	minRestRng = -1500.*2000*3.14159265359/180/65535;maxRestRng = 1000.*2000*3.14159265359/180/65535;
	#time
	prvntRndm2RstQuota = 0.1; #(_this is a guess)
	holdTrnsUp2DwnQuota = 0.5;#0.75;#0.625;
	tapSttlngQuota = 0.1;
	
		
			
	if(state == 0):
		#wait until gyrX within rest range -> transition to state 0 (rest)
		#try
		if ((gyrX>minRestRng) & (gyrX<maxRestRng)):
			timeTrck = time;	state = 0.5;
		
		#this is supposed to keep looping Random until within range (locked)
		#-------------------------------------------------------------------
	elif(state == 0.5): #Rndm2Reset
		
		#wait for prvntRndm2RstQuota (range: [-600:600] )
		if ((gyrX<minRestRng) | (gyrX>maxRestRng)):
			timeTrck = time;	state = 0;
		elif (time-timeTrck > prvntRndm2RstQuota):
			timeTrck = time;	state = 1;
		
		# mutually exclusive - order is important
		#--------------------------------------------------------------------
		
	elif(state == 1): # Rest
		
		if (gyrX > minUprThrshld):
			timeTrck = time; state = 2;
		elif (gyrX<minRestRng):
			timeTrck = time; state = 0;
		
		#-----------------------------------
		
	elif(state == 2): # triggerUp
		if (gyrX > maxUprThrshld):
			timeTrck = time; state = 0;
		elif (time-timeTrck > holdTrnsUp2DwnQuota):
			timeTrck = time; state = 0; #can consider state = 1 (rest instead of random)
		elif (gyrX < minUprThrshld):
			timeTrck = time; state = 2.5;
		
		#  -----------------------------------------------------------------
		
	elif(state == 2.5): # trnsUpDown
		if (gyrX > minUprThrshld):
			timeTrck = time; state = 2; #check this case
		elif (time-timeTrck > holdTrnsUp2DwnQuota):
			timeTrck = time; state = 0;
		elif (gyrX < minLwrThrshld):
			timeTrck = time; state = 3;
		
		
	elif(state == 3): # triggerDown
		if (gyrX < maxLwrThrshld):
			timeTrck = time; state = 0;
		elif (gyrX > minLwrThrshld):
			timeTrck = time; state = 3.5;
	
		
		
	elif(state == 3.5): # trnsDownRest
		
		if (time-timeTrck > tapSttlngQuota):
			timeTrck = time; state = 0;
			
		
	else:
		state =0 #(RandomMotion)
	
	return state
	
	