# 快递驿站取件标签生成工具（更新版）

## 1. 标签规格

- 单张标签尺寸：**80mm × 60mm**  
- 右侧 **唯一 Data Matrix**：固定 **3.2 × 3.2 cm**  
- Data Matrix 内容：完整取件码，例如 `S03-L1-0301`  
- 底部 Code128 条形码：订单号，不自带文字，数字单独绘制  
- 左侧文字区域自动缩小，不压到 Data Matrix  
- 货位说明文字固定在条形码上方

## 2. 取件码格式

```

<列>-<行>-<4位取件码>

````

- 列：货架编号，例如 `S03`  
- 行：`L1`、`L2`、`L3`…  
- 4位取件码：数字，例如 `0301`

### 示例

```txt
col,row,pickup_code,order_no,date
S03,L1,0301,2026060300030001,2026-06-03 14:28
S03,L1,0302,2026060300030002,2026-06-03 14:28
S03,L2,0311,2026060300030011,2026-06-03 14:30
S03,L3,0321,2026060300030021,2026-06-03 14:32
````

生成的完整取件码：

```
S03-L1-0301
S03-L2-0311
S03-L3-0321
```

## 3. 批量生成标签 PDF

```bash
python generate_pickup_labels.py --config config.yaml --input labels.txt --output output_labels.pdf
```

* `config.yaml`：标签样式、字体、Data Matrix 尺寸
* `labels.txt`：批量标签数据
* `output_labels.pdf`：生成的 PDF，每页 1 张标签

## 4. 拼到 A4 页面

如果想把多张标签拼到 **A4 页面**，使用 `pdf_to_a4.py`（或 PNG 拼 A4 版本）：

```bash
python pdf_to_a4.py
```

* 自动按行列排列
* 超出 A4 页自动换页
* 不依赖 Poppler 或 pdf2image

## 5. 注意事项

1. Data Matrix 只生成 **一个 3.2 × 3.2 cm**，不会拼接
2. `col` 只能 1-4 位数字或字母，`row` 格式为 L+数字
3. 单张标签 PDF 或 PNG 可以直接拼到 A4
4. PDF → PDF 拼版方法请忽略 Poppler，否则 Windows 会报错

## 6. 示例数据

`labels_30.txt` 已生成 30 条示例：

* 货架：S03
* 层：3 层
* 每层 10 个货位
* 行格式：L1、L2、L3…
* 可直接生成 30 张标签 PDF 或拼到 A4
