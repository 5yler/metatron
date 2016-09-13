## How to Create Custom Maps

This is a tutorial on creating custom maps based on real geographic data. 

1. Go to [Google Maps](https://www.google.com/maps/) and find your desired location. 
2. Take a screenshot and save the file as `<MAP_NAME>.jpg` in `gigatron/maps`
3. Create a `<MAP_NAME>.yaml` file in the same directory, and add the following contents:

```
# map file parameters
image: <MAP_NAME>.jpg
resolution: <RESOLUTION> 
origin: [-<RESOLUTION*X_ORIGIN_PIXELS>, -<RESOLUTION*Y_ORIGIN_PIXELS>, 0.0] 
negate: 0
occupied_thresh: 0.55
free_thresh: 0.35

# map origin GPS coordinates
lat: <LATITUDE>    # map origin latitude
lon: <LONGITUDE>   # map origin longitude

# parameters for static transform from map to local UTM origin
frame_id: "map" 
child_frame_id: "utm" # zone <UTM_ZONE>
x: -<UTM_EASTING>
y: -<UTM_NORTHING>
z: 0
qx: 0
qy: 0
qz: 0
qw: 1
period: 100
```

### Resolution Calculation
To get the `<RESOLUTION>` value, measure a distance in pixels between two markers on the `<MAP_NAME>.jpg` file.

![image](https://cloud.githubusercontent.com/assets/10868851/18413882/b73cd322-7784-11e6-8ed6-560a6f451c6d.png)

In this example, the measured distance is 123 pixels. Now measure the same distance on Google Maps by right-clicking on a point on the map and selecting "Measure distance":

![image](https://cloud.githubusercontent.com/assets/10868851/18413880/ab5b9eda-7784-11e6-9a6b-2c5cab798827.png)

The resolution is the distance in meters divided by the number of pixels.

### Origin Selection
Pick an arbitrary point on the map as the origin. Click on the point on Google Maps to get the `<LATITUDE>` and `<LONGITUDE>` values.

![image](https://cloud.githubusercontent.com/assets/10868851/18413916/7f5fcee0-7785-11e6-8672-ede2b41587a7.png)

Measure the distance from the lower left corner of `<MAP_NAME>.jpg` along x and y to the chosen origin. Then, fill in the following line:
```
origin: [-<RESOLUTION*X_ORIGIN_PIXELS>, -<RESOLUTION*Y_ORIGIN_PIXELS>, 0.0] 
```
Note the minus sign in both values.

### GPS to UTM Conversion
Go to [LatLong.net](http://www.latlong.net/lat-long-utm.html) and enter the latitude and longitude of the map origin into the converter.

![image](https://cloud.githubusercontent.com/assets/10868851/18413927/1ab5da6a-7786-11e6-9d55-dd73a7b4a815.png)

Now you can fill in the remaining values. Note that the `x` and `y` are the UTM easting and northing, but with a minus sign.
```
child_frame_id: "utm" # zone <UTM_ZONE>
x: -<UTM_EASTING>
y: -<UTM_NORTHING>
```


