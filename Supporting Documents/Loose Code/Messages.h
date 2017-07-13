//Header file for messeges intended to be sent around the can bus and used by all subsystems in our assembly line.
//Each should have a comment next to them for a breif explination as to wehat they do.

public bool sensorOne = false;		//Is sensor in use? false == no, true == yes
public bool sensorTwo = false;		//Is sensor in use? false == no, true == yes
public bool sensorThree = false;	//Is sensor in use? false == no, true == yes
public bool sensorFour = false;		//Is sensor in use? false == no, true == yes

public bool s_checkStatus = false;	//Has the sensor been checked sucesfully? (Maybe add a check for each individually?)
public bool beltMoving = true;		//Is the belt moving? By default it should move when the system starts, obviouslly stops when a sensor detects something.
public bool systemRunning = false; //Is the system currently running? Any type of stop should result in a false reading
static bool convayerStatus = true; //Is the convayer belt empty? We can use status values from the sensors above, true == free, false == occupied

//More needed?
