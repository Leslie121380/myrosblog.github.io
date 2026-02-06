# Transformer 文件结构
## 1. Model.py
存放基础组件：
```
clones：复制模块
LayerNorm：归一化模块
SublayerConnection：残差链接+归一化
attention：单注意力机制，计算Q·K^T/sqrt(d_k)，即给每个letter与其他letter的相关性打分,和V一起输入FNN
MultiHeadedAttention：多注意力机制，把单注意力机制的相关性尤其是与自身的相关性再次进行分散
subsequent_mask：起到遮蔽作用的上三角矩阵在训练时，掩码的效果：它会将注意力矩阵变成一个下三角矩阵。对于位置i的单词，它只能看到位置1到i的信息，而i之后的权重全部被设为负无穷，防止模型“偷看”旁边的答案
Embeddings：词嵌入（将词转换为向量表示），这个是词的本义编码
PositionalEncoding：位置编码，把每个词的位置信息也编码进来
PositionwiseFeedForward：前馈神经网络FNN
```
存放编码-解码模块：
```
EncoderLayer:单个编码器层
Encoder：N个单个编码器层的堆叠，定义堆叠后的数据流动方式，在堆顶最后一次再另进行一次归一化保证输出稳定
DecoderLayer:单个解码器层，Encoder的输入会同时给Decoder的每一层，每一层都在不断精炼翻译的语义。第一层可能只是初步匹配，而到了第六层，它已经能够结合 非常复杂的语法关系来决定当前的输出了
Decoder：单个解码器N层堆叠
Generator：词组生成器，把计算机能懂的向量变成人类能懂的单词概率
```
构成完整Encode-Decode的Transformer架构:
```
EncoderDecoder：将编码器、解码器和生成器组合在一起形成完整的Transformer
```
输入超参数实例化Transformer模型(真正的模型其实是这个`EncoderDecoder`造出来的，其他的结构都是“砖块”)：
```
def make_model(src_vocab, tgt_vocab, N=6, d_model=512, d_ff=2048, h=8, dropout=0.1):
    c = copy.deepcopy
    attn = MultiHeadedAttention(h, d_model)  # 创建多头注意力机制实例
    ff = PositionwiseFeedForward(d_model, d_ff, dropout)  # 创建位置前馈网络实例
    position = PositionalEncoding(d_model, dropout)  # 创建位置编码实例
    model = EncoderDecoder(
        Encoder(EncoderLayer(d_model, c(attn), c(ff), dropout), N),  # 创建编码器实例
        Decoder(DecoderLayer(d_model, c(attn), c(attn), c(ff), dropout), N),  # 创建解码器实例
        nn.Sequential(Embeddings(d_model, src_vocab), c(position)),  # 创建源语言嵌入层实例
        nn.Sequential(Embeddings(d_model, tgt_vocab), c(position)),  # 创建目标语言嵌入层实例
        Generator(d_model, tgt_vocab),  # 创建生成器实例
    )
```
**<big>👽Encoder:<big>**

Embedding and PositionalEncoding ➡️ Multi-Head Attention(生成Q,K,V矩阵，计算Q×K生成每个词的关联性打分) ➡️ LayerNorm(x+Sublayer(x))先Dropout再加原始输入（残差连接）再一起归一化 ➡️ FNN ➡️ 再次残差连接归一化

**👽<big>Decoder:<big>**

✈️编码后的内容存储在memory一起里面送入到Decoder的每一层，所以实际上每一层都会得到整个句子的所有信息，这也是我们需要做掩码处理的原因，以下步骤是Decoder的每一层的处理结构,不断循环✈️

计算Q,K,V ➡️ 应用subsequent_mask进行掩码处理 ➡️ Q,K点积计算相关性分数（了解每个词的关联性）➡️ FNN ➡️ Do it all over again ➡️ 最后一层处理完后其实返回的是一个概率表 ➡️ Generator ➡️选择概率最大的预测词输出

🔦深层提炼：Decoder 的每一层都在不断精炼翻译的语义。第一层可能只是初步匹配，而到了第六层，它已经能够结合非常复杂的语法关系来决定当前的输出了。

🔦防止信息丢失：Transformer 堆叠很深，如果只在底层注入一次，源句子的信息在向上传递过程中会逐渐模糊。每一层都注入，相当于在每个处理阶段都“翻看一遍原件”，确保翻译不偏离主题。

## 2. train.py
```
Batch:用于对生成的随机源序列中的每个元素进行覆盖掩码的操作
run_epoch：传入词源，词源处理后的矩阵，掩码，model。输入进行模型的训练，得到损失函数
```

## 3. main.py
```
调用`model.py`中的make_model和subsequent_mask生成模型和掩码
data_gen生成随机序列➡️Batch处理和覆盖掩码➡️run_epoch模型训练➡️训练后的模型输入greedy_decode预测序列
```

🦩Attention!!!greedy_decode送入的是我们的model，源序列，空mask（后面会对应每个元素生成对应的mask），长度和起始元素），

然后进去之后进行encode,decode的操作得出的out其实是一个概率（选哪个作为下一个预测输出的概率），然后使用最大概率的作为我的

输出并拼接上去*。


 

   





 

