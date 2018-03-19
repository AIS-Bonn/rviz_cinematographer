# General

Collection of packages for the thesis of Jan Razlaw with the working title "Detection and Tracking of Small Objects in Sparse 3D Laser Range Data".

The goal of the thesis is to map the static part of the environment while detecting and tracking dynamic objects of a specified width.

# Packages

**laser_segmentation** : segments each scan ring of a velodyne scan with respect to a specified width

**map_handler** : stores a representation of the environment consisting of a static and a dynamic part

**object_detector** : detects objects in the map

**multi_object_tracker** : tracks detected objects and provides dynamics to map_handler