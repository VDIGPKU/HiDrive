#!/bin/bash
killall -9 -r 'CarlaUE4|CarlaUnreal' > /dev/null 2>&1 || true
ps -ef | grep "carla-rpc-port" | grep -v grep | awk '{print $2}' | xargs -r kill > /dev/null 2>&1 &
ps -ef | grep "run_evaluation" | grep -v grep | awk '{print $2}' | xargs -r kill > /dev/null 2>&1 &
ps -ef | grep "leaderboard_evaluator" | grep -v grep | awk '{print $2}' | xargs -r kill > /dev/null 2>&1 &
wait
