# EduKit-Team-Q



EdgeKit -> IotCore -> Iot Rule -> Lambda -> Mock Service


Suncorp Sensorthon project, Team Q

AWS IoT Core, Lab 1 - https://iot.awsworkshops.com/aws-iot-core/lab1-gettingstarted/

https://ap-southeast-2.console.aws.amazon.com/console/home?region=ap-southeast-2#

https://github.com/MChoc/Sensorthon
in particular the workshop: https://github.com/MChoc/Sensorthon/tree/main/Workshop



## Message Formats

Ping message

EduKit_Q/pub

    {
        "ThingInformation": {
            "time": 4830161,
            "team": "Q"
        },
        "message": "Hello, this is transmitting from the Edukit"
    }


EduKit_Q/imu

    {
        "ThingInformation": {
            "time": 4830161,
            "team": "Q"
        },
        "IMU": {
            "accX": 1.0,
            "accY": 1.0,
            "accZ": 1.0,
            "gyroX": 1.0,
            "gyroY": 1.0,
            "gyroZ": 1.0,
            "pitch": 1.0,
            "roll": 1.0,
            "yaw": 1.0,
            "temp": 54.454
        }
    
    }

EduKit_Q/motion


    {
        "ThingInformation": {
            "time": 4830161,
            "team": "Q"
        },
        "Motion": {
            "value": 43.5
        }
        
    }
