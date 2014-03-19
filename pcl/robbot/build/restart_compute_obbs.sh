#!/bin/sh
RC=1
while [ $RC -ne 0 ]; do
   ./compute_obbs ../../dataset/rgbd-dataset/
   RC=$?
done

