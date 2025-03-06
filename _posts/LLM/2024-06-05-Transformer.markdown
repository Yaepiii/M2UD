---
title: "Transformer"
layout: post
date: 2024-06-05 21:47
image: /assets/images/markdown.jpg
headerImage: ture
tag:
- markdown
- components
- extra
hidden: true
category: LLM
author: Zhenshuai Yin
description: 记录自己对Transformer的理解

---

参考: [【超详细】【原理篇&实战篇】一文读懂Transformer-CSDN博客](https://blog.csdn.net/weixin_42475060/article/details/121101749)

[【官方双语】GPT是什么？直观解释Transformer | 深度学习第5章 (3blue1brown)](https://www.bilibili.com/video/BV13z421U7cs?spm_id_from=333.880.my_history.page.click)

# Transformer

Transformer是一种用于自然语言处理(NLP)和其他序列到序列(seq-to-seq)任务的深度学习模型架构, 它在2017年由Vaswani等人提出(好早). Transformer架构引入了自注意力机制(self-attention mechanism), 这使其在处理序列数据时表现出色.

>  Transformer的重要组成部分和特点:
>
> ​	自注意力机制: 它使模型能够同时考虑输入序列中的所有位置, 而不是像循环神经网络(RNN)或卷积神经网络(CNN)一样逐步处理.自注意力机制允许模型根据序列中的不同部分来赋予不同的注意权重, 从而更好地捕捉语义关系.
>
> ​	多头注意力: Transformer中的自注意力机制被扩展为多个注意力头, 每个头可以学习不同的注意权重, 以更好地捕捉不同类型的关系. 多头注意力允许模型并行处理不同的信息子空间.
>
> ​	堆叠层: Transformer通常由多个相同的编码器和解码器堆叠而成, 这些堆叠的层有助于模型学习复杂的特征表示和语义.
>
> ​	位置编码: 由于Transformer没有内置的序列位置信息, 它需要额外的位置编码来表示输入序列中单词的位置顺序.
>
> ​	残差连接和层归一化
>
> 编码器和解码器: Transformer通常包括一个编码器用于处理输入序列和一个解码器用于生成输出序列, 这使其适用于序列到序列的任务, 如机器翻译.

## Transformer的结构

![在这里插入图片描述](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/5d74c1e4fd7c435a914778542258b1de.png)

Transformer中, Encoder block由6个Encoder堆叠而成, 每个编码/解码器在结构上是相同的, 但它们之间没有共享参数, 编码解码器的简略结构如下:

![在这里插入图片描述](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/352a82d75c6447d984ad6981a9fc0121.png)

从编码器输入的句子经过Embedding(转换为词向量)之后, 会经过自注意力层, 这一层使得句子中的每个token学习到上下文知识, 关注到句子中的其他单词(这一点在3blue1brown中, 展示的非常棒).  解码器中的解码注意力层的作用是关注输入句子的相关部分(这啥意思).

## 自注意力机制

自注意力的作用: 随着模型处理输入序列的每个单词, 自注意力会关注整个输入序列的所有单词, 改变token序列中词向量的位置(相当于改变其语义, 因为结合了上下文的信息).

​	序列建模: 自注意力可以用于序列数据的建模. 它可以捕捉序列中不同位置的依赖关系, 从而更好地理解上下文(在代码上的实现手段就是通过矩阵Q*矩阵K, 得出的点积(代表相似性)越大, 就表明这两个token之间(表达有问题, 是单向的)的相关性大, 从而占得权重大).

​	并行计算: 因为Transformer中使用的计算手段基本上都是矩阵乘法, 因此可以使用并行计算, 更容易在GPU和TPU等硬件上进行高效的训练和推理.

​	长距离依赖捕捉: 传统的循环神经网络(RNN)在处理长序列时可能面临梯度消失或梯度爆炸的问题. 自注意力机制可以更好地处理长距离依赖捕捉(甚至Transformer中不会根据位置的远近而有任何改变, 当然为了解决这一问题, 在token转换为词向量时, 在其中加入了位置信息)

![在这里插入图片描述](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/f146c2a912a148bba6b3edd0175e7f48.png)

### 自注意力的计算(之后我要用3blue1brown的图替换)

Transformer的每个自注意力层拥有三个矩阵: 查询矩阵W^Q^,键值矩阵W^K^,值矩阵W^V^

在输入的token序列经过Embedding之后, 每个token会转化为一列高维的词向量(这个词向量对每个token是固定的)(词向量的具体值是通过训练得出的, 词义相近的词互相接近(体现为点积为正, 越大越接近))

将词向量分别于查询矩阵W^Q^和键值矩阵W^K^相乘, 可以得出两个高维向量, 体现为一个表, 啊啊啊啊, 放弃了, 等下周好好整理一下3blue1brown的图再写blog了

![image-20240606001718870](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/image-20240606001718870.png)

![image-20240606001825524](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/image-20240606001825524.png)

哥们决定了, 还是好好看论文原文吧, 这种吃二手blog太难受了, 完全不知道自己理解的是不是正确的意思, 也不知道blog写的是否正确.











































