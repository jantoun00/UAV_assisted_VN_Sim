# UAV_assisted_VN_Sim
Discrete event simulator for UAV assisted Vehicular network Scenario 
In this project we simulate a scenario of a Highway segment where there is a stream of vehicle traffic flowing on it and these vehicles have a single packet to transmit to a destination RSU.
The vehicles arrival follows a poisson process.
The vehciles form clusters while traveling along the highway. 
A flow of UAVs is dispatched over the following highway to help establish a full link to the destination RSU.
Once a link is established the packets are sent to the destination RSU.
This project is build using the Simpy library in Python.
