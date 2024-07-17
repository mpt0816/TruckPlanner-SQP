# Truck Path Planner

A Tractor-Trailer Path Planner Algorithm. 

This is the code reproduction of paper “Optimization-Based On-Road Path Planning for Articulated Vehicles”，which plans a path for semi-trailer trucks. And The specific algorithm details are described in paper "Path Planning for Autonomous Bus Driving in Urban Environments".

## 1. Install osqp

Run `install_osqp.m`.

## 2. Run program

Run `Main.m`.

## 3. View results

Figures of program is in `TruckPathPlanner/figure`.

Results of `Senarios # 12`:

## 3.1 Final Planned Path Animation 

![](https://github.com/mpt0816/TruckPlanner-SQP/blob/main/figure/scenario%2312%23S-turn.gif)

## 3.2 Planned Path of SQP  iterative process

![](https://github.com/mpt0816/TruckPlanner-SQP/blob/main/figure/scenario%2312%23S-turn-IterPathPlan.png)

## 3.3 The Variables of Final Planned Path 

![](https://github.com/mpt0816/TruckPlanner-SQP/blob/main/figure/scenario%2312%23S-turn-Variables.png)

# 4. Scenario

Edit the code in folder `scenario` to generate scenarios. 
