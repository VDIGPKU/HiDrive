# HiLevAD Route Splits

The current full benchmark file is:

```text
leaderboard/data/HLADs.xml
```

It contains 330 routes. The current A/B/C split is:

| Split | Route Count |
| --- | ---: |
| A | 209 |
| B | 75 |
| C | 46 |
| All | 330 |

## Route Blocks

Contiguous routes with the same scenario label are grouped as blocks. If the
same scenario appears in non-contiguous regions, it is listed as separate
blocks.

| Routes | Scenario | Split |
| --- | --- | --- |
| route1-24 | SimplePedestrianCrossing | A |
| route25-49 | SimpleBicycleCrossing | A |
| route50-61 | LeadVehicleBrake | A |
| route62-91 | BrokenDownVehicle | A |
| route92-131 | CenterBarrierObstacle | B |
| route132-138 | HighwayCutIn | A |
| route139-144 | LeadVehicleBrake | A |
| route145-150 | ParkingPullOut | A |
| route151-151 | SignalizedJunctionLeftTurnEnterFlow | A |
| route152-152 | SignalizedJunctionRightTurn | A |
| route153-154 | SignalizedJunctionLeftTurnEnterFlow | A |
| route155-156 | SignalizedJunctionRightTurn | A |
| route157-158 | SignalizedJunctionLeftTurnEnterFlow | A |
| route159-159 | SignalizedJunctionRightTurn | A |
| route160-163 | HighwayCutIn | C |
| route164-167 | RoadsideMergeEthicsMonitor | A |
| route168-191 | NarrowPassage | A |
| route192-192 | NarrowPassageFollowingFront | A |
| route193-193 | NarrowPassageFollowingBoth | A |
| route194-194 | NarrowPassageFollowingFront | A |
| route195-195 | NarrowPassageFollowingBoth | A |
| route196-196 | NarrowPassageFollowingFront | A |
| route197-197 | NarrowPassageFollowingBoth | A |
| route198-198 | NarrowPassageFollowingFront | A |
| route199-199 | NarrowPassageFollowingBoth | A |
| route200-200 | NarrowPassageFollowingFront | A |
| route201-201 | NarrowPassageFollowingBoth | A |
| route202-202 | NarrowPassageFollowingFront | A |
| route203-203 | NarrowPassageFollowingBoth | A |
| route204-204 | NarrowPassageFollowingFront | A |
| route205-205 | NarrowPassageFollowingBoth | A |
| route206-206 | NarrowPassageFollowingFront | A |
| route207-207 | NarrowPassageFollowingBoth | A |
| route208-208 | NarrowPassageFollowingFront | A |
| route209-209 | NarrowPassageFollowingBoth | A |
| route210-210 | NarrowPassageFollowingFront | A |
| route211-211 | NarrowPassageFollowingBoth | A |
| route212-212 | NarrowPassageFollowingFront | A |
| route213-213 | NarrowPassageFollowingBoth | A |
| route214-214 | NarrowPassageFollowingFront | A |
| route215-215 | NarrowPassageFollowingBoth | A |
| route216-216 | NarrowPassageFollowingFront | A |
| route217-217 | NarrowPassageFollowingBoth | A |
| route218-218 | NarrowPassageFollowingFront | A |
| route219-219 | NarrowPassageFollowingBoth | A |
| route220-220 | NarrowPassageFollowingFront | A |
| route221-221 | NarrowPassageFollowingBoth | A |
| route222-222 | NarrowPassageFollowingFront | A |
| route223-223 | NarrowPassageFollowingBoth | A |
| route224-224 | NarrowPassageFollowingFront | A |
| route225-225 | NarrowPassageFollowingBoth | A |
| route226-226 | NarrowPassageFollowingFront | A |
| route227-227 | NarrowPassageFollowingBoth | A |
| route228-228 | NarrowPassageFollowingFront | A |
| route229-229 | NarrowPassageFollowingBoth | A |
| route230-230 | NarrowPassageFollowingFront | A |
| route231-231 | NarrowPassageFollowingBoth | A |
| route232-232 | NarrowPassageFollowingFront | A |
| route233-233 | NarrowPassageFollowingBoth | A |
| route234-234 | NarrowPassageFollowingFront | A |
| route235-235 | NarrowPassageFollowingBoth | A |
| route236-236 | NarrowPassageFollowingFront | A |
| route237-237 | NarrowPassageFollowingBoth | A |
| route238-238 | NarrowPassageFollowingFront | A |
| route239-239 | NarrowPassageFollowingBoth | A |
| route240-251 | SlowLeadVehicle | A |
| route252-257 | BrokenDownVehicle | C |
| route258-263 | WeaveVehicle | B |
| route264-272 | PuddleStandingPedestrians | B |
| route273-276 | VehicleOpensDoorNoFlow | B |
| route277-280 | FiretruckPuddleTrail | C |
| route281-284 | CarPuddleTrail | C |
| route285-292 | YieldToEmergencyVehicle | B |
| route293-294 | MalfunctionSignalJunctionLeftTurnEnterFlow | C |
| route295-295 | MalfunctionSignalJunctionRightTurn | C |
| route296-300 | BrakeFailureDilemma | C |
| route301-304 | PoliceInterceptStop | B |
| route305-308 | BrakeFailure | B |
| route309-309 | WrongWayVehicle | C |
| route310-310 | RearEndPause | C |
| route311-311 | WrongWayVehicle | C |
| route312-312 | RearEndPause | C |
| route313-313 | WrongWayVehicle | C |
| route314-314 | RearEndPause | C |
| route315-315 | WrongWayVehicle | C |
| route316-316 | RearEndPause | C |
| route317-318 | NoScenario | A |
| route319-330 | CameraOcclusion | C |
