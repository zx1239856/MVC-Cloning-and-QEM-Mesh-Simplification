# MVC Cloning and Mesh Simplification
## MVC Cloning
### Usage
```
.\mvc --src [SRC_PATCH] --dst [DST_PATCH] --out [OUT_FILENAME] --mask [MASK_FILE] --interactive --x [X_POS] --y [Y_POS]
```s
When `--interactive` is present, interactive mode is on, where you can create mask/ select the destination position of patch via GUI, otherwise in CLI mode you need to specify `--x` and `--y` position. 
Note that you do not need to create mask if you've specified mask file through `--mask`.
### Demo
![image](./demo/MVC/result/bear.png)

## Mesh Simplification
Based on Quadric Error Metrics
### Usage
```
.\main --src [SRC_OBJ_FILE] --dst [OUTPUT_OBJ_FILE] --ratio [SIMPLIFICATION_RATIO]
```
Note the simplification ratio is based on vertex numbers. But in our experiments we found it applicable to triangular face numbers too. 
***Only OBJ model is supported (we didn't implement UV coordinate here).***
### Demo
![image](./demo/Mesh/bunny00.png)
![image](./demo/Mesh/bunny05.png)
