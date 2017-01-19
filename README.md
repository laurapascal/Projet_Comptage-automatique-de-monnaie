# Change counter project

This program can be used to automatically count the amount of change placed on a flat surface by detecting and recognizing each coins.

## Program usage

```
    Option:
    	--database: <std::string> database folder path
    	--detection: <int> detection method number: 1, 2 or 3
    		1: detection of circles thanks to Hough transform
    		2: detection of circles or ellipses thanks to threshold and contour detection
    	--features_detection: <std::string> features detection algorithm: 'sift' ,'surf' or 'orb'
    	--matcher: <std::string> matcher algorithm: 'flann' or 'BF'
    	--debug: <std::string> display debug images: 'all', 'none', '1', '2', or '3'
    		all: display debug for all steps
    		none: doesn't display any debug
    		1: display debug for circle destection step
    		2: display debug for registration step
    		3: display debug for score computation step
    	--score: <int> score computing method: 1 or 2
    		1: compute the score with the number of inliers found
    		2: add of a weighting compute with the repartition of the inliers found
    		3: compute the score with template matching
    	--size: <int> size used for the resize (of the database and the extracted coins) before the comparison: between 250 and 1000	
    \nOption Values by default: --database DataBase1000 --detection 2 --features_detection orb --matcher BF --debug none --score 1 --size 250

```

## Data base

The database must be composed as folowing:
- A source folder containing one folder per coin
- Each coin folder must be named XXc for the cents and XXe for the euros, where XX is the value of the coin
- Each coin folder will contain as many coins images as wanted (the bigger the database is the slower the program will be)

## Resources

A database and some test images are available here: [ressources](https://drive.google.com/drive/folders/0B36RmGk5sPzIR0ljczJleVRJZk0)

## Authors

### Laura Pascal
### Jean-Baptiste Vimort
