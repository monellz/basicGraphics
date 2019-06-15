# Surface Simplification

实现 *Surface Simplification Using Quadric Error Metrics*

* 采用hafl edge结构
* 使用第三方库 tiny\_obj\_loader.h
* 不支持之前未连接的顶点合并
* 只支持manifold的obj

## Usage

```bash
make
./main input.obj output.obj ratio
#ratio: target face num / total face num
```

## Record

- [x] he结构 19.06.15
- [x] pair selection算法优化 19.06.15
