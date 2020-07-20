# Range-Image-based-segementation
An implementation on "I. Bogoslavskyi and C. Stachniss, "Fast range image-based segmentation of sparse 3D laser scans for online operation," 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, pp. 163-169, doi: 10.1109/IROS.2016.7759050."

An official implementaiton is here:https://github.com/PRBonn/depth_clustering 

The ground remove code is from https://github.com/AbangLZU/plane_fit_ground_filter. We achieve multi-plane fitting by using multi-thread.

### Different from paper:

    1. We changed the ground segmentation method
    2. Using hash map replace the image-based search
    
To the horizon neighbor search, we didn't use the method in paper. The method that we used to classify two neighbor objects as follows: 

![Image text](https://github.com/WAN96/Range-Image-based-segementation/blob/master/image/threshood.jpeg)

    
    
### How to use

    mkdir build
    cd build
    cmake ..
    make
    ./range_cluster test.bin

### The Result

![Image text](https://github.com/WAN96/Range-Image-based-segementation/blob/master/image/cloud.png)
![Image text](https://github.com/WAN96/Range-Image-based-segementation/blob/master/image/range_image_only.png)
![Image text](https://github.com/WAN96/Range-Image-based-segementation/blob/master/image/range_image_with_cluster.png)
![Image text](https://github.com/WAN96/Range-Image-based-segementation/blob/master/image/ouput2.png)


