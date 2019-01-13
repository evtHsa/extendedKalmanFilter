# **Extended Kalman  Filter Project** 

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the provided simulator to get radar and laser(lidar) data to test the implementation of Extended Kalman Filters primarily based on the limited implementations we were guided through in lesson 25 "Extended Kalman Filters" and the information provided in the instructions for "Extended Kalman Filters Project"
	* detailed citations of the source for code fragments for the TODOs are given by comments in the source
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./writeup_images/image1.jpg " "


## Rubric Points
### Code must compile without error with cmake and make
It does

---
### Accuracy
#### Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].
It does.
![alt text][image1]
### Follow the correct algorithm
As indicated in the opening of this document, we followed the suggested path from the lessons

### Code Efficiency
Where possible duplicate code was factored out to reduce the risk of neglecting to fix every place where a change needed to be made. Although it is a bit more elegant, there was not much effect in practice as the code from the lessons was clean.

Common subexpressions were moved to temporary variables. The utiliity of this was questionable in light of modern compiler technology.

### Running the Project

 - run cmake and make per the project instructions
 - start the simulator(requires, per the instructions, installation of the term2 simulator)
 - start build/ExtendedKF

### Closing Note
I struggled for a while to understand why my RMSE values were not within spec until I revisited the "Tips and Tricks" and implemented the suggestion to normalize the angles resultant from atan2. Thank you!!
