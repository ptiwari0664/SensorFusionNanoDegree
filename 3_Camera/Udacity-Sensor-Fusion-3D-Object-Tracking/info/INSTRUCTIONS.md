# FP.1 : Match 3D Objects

In this task, please implement the method "matchBoundingBoxes", which takes as input both the
previous and the current data frames and provides as output the ids of the matched regions of
interest (i.e. the boxID property)“. Matches must be the ones with the highest number of keypoint
correspondences.

The task is complete once the code is functional and returns the specified output, where each
bounding box is assigned the match candidate with the highest number of occurrences.

# FP.2 : Compute Lidar-based TTC

In this part of the final project, your task is to compute the time-to-collision for all matched 3D
objects based on Lidar measurements alone. Please take a look at the second lesson of this course to
revisit the theory behind TTC estimation. Also, please implement the estimation in a way that makes
it robust against outliers which might be way too close and thus lead to faulty estimates of the
TTC. Please return your TCC to the main function at the end of computeTTCLidar.

The task is complete once the code is functional and returns the specified output. Also, the code is
able to deal with outlier Lidar points in a statistically robust way to avoid severe estimation
errors.

# FP.3 : Associate Keypoint Correspondences with Bounding Boxes

Before a TTC estimate can be computed in the next exercise, you need to find all keypoint matches
that belong to each 3D object. You can do this by simply checking wether the corresponding keypoints
are within the region of interest in the camera image. All matches which satisfy this condition
should be added to a vector. The problem you will find is that there will be outliers among your
matches. To eliminate those, I recommend that you compute a robust mean of all the euclidean
distances between keypoint matches and then remove those that are too far away from the mean.

The task is complete once the code performs as described and adds the keypoint correspondences to
the "kptMatches" property of the respective bounding boxes. Also, outlier matches have been removed
based on the euclidean distance between them in relation to all the matches in the bounding box.

# FP.4 : Compute Camera-based TTC

Once keypoint matches have been added to the bounding boxes, the next step is to compute the TTC
estimate. As with Lidar, we already looked into this in the second lesson of this course, so you
please revisit the respective section and use the code sample there as a starting point for this
task here. Once you have your estimate of the TTC, please return it to the main function at the end
of computeTTCCamera.

The task is complete once the code is functional and returns the specified output. Also, the code
must be able to deal with outlier correspondences in a statistically robust way to avoid severe
estimation errors.

# FP.5 : Performance Evaluation 1

This exercise is about conducting tests with the final project code, especially with regard to the
Lidar part. Look for several examples where you have the impression that the Lidar-based TTC
estimate is way off. Once you have found those, describe your observations and provide a sound
argumentation why you think this happened.

The task is complete once several examples (2-3) have been identified and described in detail. The
assertion that the TTC is off should be based on manually estimating the distance to the rear of the
preceding vehicle from a top view perspective of the Lidar points.

# FP.6 : Performance Evaluation 2

This last exercise is about running the different detector / descriptor combinations and looking at
the differences in TTC estimation. Find out which methods perform best and also include several
examples where camera-based TTC estimation is way off. As with Lidar, describe your observations
again and also look into potential reasons. This is the last task in the final project.

The task is complete once all detector / descriptor combinations implemented in previous chapters
have been compared with regard to the TTC estimate on a frame-by-frame basis. To facilitate the
comparison, a spreadsheet and graph should be used to represent the different TTCs.

We are super-excited to receive your submission as soon as possible.
