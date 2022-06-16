# EduKit-Team-Q

EdgeKit -> IotCore -> Iot Rule -> Lambda -> Mock Service

Suncorp Sensorthon project, Team Q

AWS IoT Core, Lab 1 - https://iot.awsworkshops.com/aws-iot-core/lab1-gettingstarted/

AWS Console: https://ap-southeast-2.console.aws.amazon.com/console/home?region=ap-southeast-2#

Sensorthon Github: https://github.com/MChoc/Sensorthon
Sensorthon Workshop in Github: https://github.com/MChoc/Sensorthon/tree/main/Workshop

Things Webapp: https://dev3553.d3ewme81sefllk.amplifyapp.com/things

# Lambda

Lambda Created to send notifications (Notifications can be seen here https://dev3553.d3ewme81sefllk.amplifyapp.com/things) - Select team Q

Lambda has been attached to the rule https://ap-southeast-2.console.aws.amazon.com/iot/home?region=ap-southeast-2#/rule/sensorthon_team_q_imu and more rules can be added under Message Routing -> Rules

The rule can be triggered by events published.

# Message Formats

Ping message

`EduKit_Q/pub`

    {
        "ThingInformation": {
            "time": 4830161,
            "team": "Q"
        },
        "message": "Hello, this is transmitting from the Edukit"
    }


`EduKit_Q/imu`

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

`EduKit_Q/motion`

    {
        "ThingInformation": {
            "time": 4830161,
            "team": "Q"
        },
        "Motion": {
            "value": 43.5
        }
        
    }
