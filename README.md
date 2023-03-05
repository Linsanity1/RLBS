# Receiver-Localization-and-Beam-Steering

This project is the implementation of our proposed scheme of receiver localization and beam steering. With the 3D printing models and the source code, you can replicate the findings in our paper.

<img src="./Assets/Images/stand_and_receiver.png"  width="70%" height="70%">

This project includes:
- source code (in python) that implements the functionalities and the experimental protocols
- 3D-printing models (STL files) for the mechanical design
- PCB design for the receiver
- experimental results in video form
- bill of materials

## Project structure
```
├── Code
│   ├── main.py                       <- main file implementing the control logic described in the paper
│   ├── protocol_1.py                 <- file for experimental protocol 1
|   ├── protocol_2.py                 <- file for experimental protocol 2
│   └── Utils
│   │   ├── PID.py                    <- PID controller class
│   │   ├── ReceivedPoints.py         <- class for discerning the receiver from the captured 5 points
│   │   ├── StepperMotor.py           <- motor controller class
│   │   └── tracker.py                <- tracker class
├── Hardware
│   ├── 3D-Printing Models
│   └── Printed Circuit Board
├── Assets
│   ├── Images
│   └── Videos
└── README.md
```

## Preparation
- To get started, you need to have two stepper motors, two motor controllers, Raspberry Pi 4B+ with Camera module v2 (or other computers and cameras, but the code needs modification), an infrared filter, an infrared laser (e.g., 100 mW, 808nm), and two laser mirros.
- Manufacture the 3D-printing models and the PCB, and assemble everything.
- Setup the python environment on Raspberry Pi. See [Ref](https://www.youtube.com/watch?v=QzVYnG-WaM4&t=2s).
- Test the system performance with `main.py`, `protocol_1.py`, or `protocol_2.py`.

## Experimental Videos
**Protocol 1**

<img src="./Assets/Videos/protocol_1.gif"  width="30%" height="30%">

Parameters: the distance between the transmitter and the receiver is 2 m; the initial distance between the receiver and the laser spot is 10 cm.

**Protocol 1 with 256 microstepping**

<img src="./Assets/Videos/protocol_1_256_microstep.gif"  width="30%" height="30%">

**Protocol 2 with 1 cm/s receiver speed**

<img src="./Assets/Videos/1cm_s.gif"  width="30%" height="30%">

**Protocol 2 with 5 cm/s receiver speed**

<img src="./Assets/Videos/5cm_s.gif"  width="30%" height="30%">

**Protocol 2 with 10 cm/s receiver speed**

<img src="./Assets/Videos/10cm_s.gif"  width="30%" height="30%">

## Bill of Materials
For researchers in China, you can purchase the components in the following links:
- [Stepper Motor](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8d1TXj6z&id=600124844336&_u=528s2ekd5bd1)
- [Motor Controller](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8d1TXj6z&id=673302946671&_u=528s2ekd75e7)
- [Raspberry Pi 4B+](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8dekqXXo&id=596761703325&_u=528s2ekdece8)
- [Raspberry Pi Camera Module v2](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8dekqXXo&id=531388836999&_u=528s2ekdae12)
- [Infrared Filter](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8d39Q22d&id=41482304810&_u=528s2ekdcfbc)
- [Infrared Laser](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8d1TXj6z&id=597192207671&_u=528s2ekd5970)
- [Laser Mirror (circular)](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8d1TXj6z&id=586249224446&_u=528s2ekddfc6)
- [Laser Mirror (rectangular)](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.52632e8d1TXj6z&id=577756381316&_u=528s2ekdd99e)
- [PCB Manufacturing](https://www.jlc.com/)
- [3D Printing](https://www.sanweihou.com/)

For researchers outside China, you can purchase similar components on other platforms, and feel free to contact me if you have any question.

## 
