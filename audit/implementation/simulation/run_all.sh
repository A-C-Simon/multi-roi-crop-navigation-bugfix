#!/usr/bin/env bash
R=/tmp/mr_sim/run_case.sh
G=/tmp/mr_sim/worlds/farm_straight_gaps.world
$R straight_map 300 field:=straight max_seconds:=110 lookahead:=true
$R curve_nomap 300 field:=curve max_seconds:=110
$R curve_map 300 field:=curve max_seconds:=110 lookahead:=true
$R gaps_nomap 300 field:=custom world:=$G max_seconds:=110
$R gaps_map 300 field:=custom world:=$G max_seconds:=110 lookahead:=true
$R line_straight 300 field:=straight max_seconds:=110 line_fit:=true
echo ALLDONE
