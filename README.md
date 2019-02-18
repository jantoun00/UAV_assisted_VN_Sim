# UAV_assisted_VN_Sim
This project is build using the Simpy library in Python.<br>
Discrete event simulator for UAV assisted Vehicular network Scenario.<br>
In this project we simulate a highway segment that have a vehicular traffic flow that form a part of a vehicular network, every vehicle have a packet to transmit to a destination RoadSide Unit (RSU).<br>
A flow of UAVs is added to the vehicular flow to form additional node for the multi-hop path between source and destination RSU. Once a path is established the available packets are sent to the destination RSU.<br>
The average packet delay and end-to-end path availability is calculated at the end of each simulation run. <br>
The vehicules arrivals and UAVs follows a poisson arrival process. <br>
This is the first version of this simulator, further improvement will be applied and followed by a patch note to explain the changes done.<br>


