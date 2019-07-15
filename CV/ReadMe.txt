This application allows to compare two images and find connection points. Afterwards it performs homography calculation and connection of two images. It is consist of four parts: 

1. Harris matrix and it's resolve function computation with KeyPoints vector output (GetKeyPoints function inside Descriptor class). In the project description it was required to output the image 1b and 1c for this part, but I omit it since I used the results from my A2 where it worked fine - and here for the multiple images calculation writing images caused some bugs I couldn't track completely. However for the basic output those line can be de-commented for the purpose of project description.

2. Matching points for the two images:
Main difference with the A2 - now ratio test removing ambigious results from the output list. Descriptor.GetMatches(...) method returns vector of DMatches as well as 2D vector of Point2f pairs. ImWrite for this method are also commented but can be used to write basi output. I recomment to not decomment it since it causes type mismatch bugs sometimes.

3. In third part MyStitcher class, method RANSAC performs calculation of best possible homography, using computeInliers and project methods respectively. In the end it chooses best homography, recalculates it for all inliers, calculates it's inverse and writes an image with only this inliers used to display connections. (3.png).

4. Last part of logic, stitch() method connects 2 images projecting image 2 on image one, calculating new area to copy image one, then projecting new are on image 2 - deciding either to copy image 2 pixel if it's exist or blend it if original pixel also not 0. For blending alpha blending were used (color chosen based on distance from image 2 center.).

5. In main() method of the program there are 3 methods used:
basic() - computating stitching for the Rainer1 and Rainer2 images.
multiple() - for all Rainer images (1-6). Commented in default.
multipleMy() - for my images taken on my camera. Also commented. Also takes more time to calculate.

So in the end, basic requirements and extra 1 and 2 were completed for this project. 