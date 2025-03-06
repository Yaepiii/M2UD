---
title: "Parameter-Efficient Fine-Tuning"
layout: post
date: 2024-06-04 23:43
image: /assets/images/markdown.jpg
headerImage: ture
tag:
- markdown
- components
- extra
hidden: false
category: LLM
author: Zhenshuai Yin
description: 记录自己对于Parameter-Efficient Fine-Tuning的理解
---

[toc]

参考:[Prompt-Tuning——深度解读一种新的微调范式_prompt tuning-CSDN博客](https://blog.csdn.net/qq_36426650/article/details/120607050)

本人只是转载, 在作者的基础上加上一些自己的理解和侧重点

因参考作者用Prompt Tuning代表Parameter-Efficient Fine-Tuning, 故本文中Prompt Tuning有时表示Prompt Tuning, 有时表示Parameter-Efficient Fine-Tuning.

LLM的发展历程:

![在这里插入图片描述](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/1614c2f147bf4ed4a4cee9af5c751a46.png)

面向LLM的Prompt-Tuning的发展历程:

![在这里插入图片描述](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/9022caf1c4a74c75942539454f1b689f.png)

自从GPT等LLM相继提出, 以Pre-training+Fine-Tuning的模式在诸多NLP任务中被广泛应用: 首先在pretraining阶段通过在大规模无监督语料上的训练得到一个预训练语言模型(Pre-trained Language Model, PLM, 同样也是GPT中的P,即pretrained), 然后再Fine-tuning阶段基于训练好的语言模型在具体的下游任务中进行微调, 以获得适应下游任务的模型.

存在的问题: 在大多数的微调中, 下游任务的目标与预训练目标差距过大导致微调效果不明显;

​					微调过程中依赖大量的监督语料(即问答对数据集)

因此: 以GPT-3, PET为首提出了一种基于预训练语言模型的新的微调范式: Prompt-Tuning, 其旨在通过添加模版的方式来避免引入额外的参数, 从而让语言模型可以在小样本(Few-shot)或零样本(Zero-shot)场景下达到理想效果

Prompt-Tuning又称Prompt, Prompting, Prompt-based Fine-tuning等

Prompt Tuning的动机: 

​	降低语义差异(预训练和微调阶段)

​	避免过拟合: 由于在Fine-tuning阶段需要引入额外参数, 并且样本数量有限, 因此容易发生过拟合, 降低模型的泛化能力.

# 预训练语言模型

以GPT,ELMO, BERT为首的预训练语言模型在近两年大放异彩, 主要分为两种类型:

​	单向: 以GPT为首, 强调从左向右的编码顺序, 适用于Encoder-Decoder模式的自回归, 不会让后面的词影响到前面的词, 在Transformer的Attention部分会将后面词对前面词的影响使用掩码遮盖住

​	双向: 以ELMO为首, 强调双向编码, 但ELMO的主题是LSTM, 以串行进行编码, 无法并行, 因此最近以BERT(以Transformer为主体结构)作为双向语言模型的基准

## Transformer

Transformer模型是由谷歌的机器翻译团队在2017年末提出的(没错, 就是翻译团队), 是一种完全利用Attention机制构件的端到端模型.

Transformer的优势在于, 其推理完全由矩阵乘法实现, 实现了并行运算, 可以加速推理过程, 并且能够很好的体现出上下文对词义的影响(这个可能别的模型也可以)

> 在NLP领域中, Attention机制的目标是对具有强相关的token提高模型的关注度. 例如在文本分类中, 部分词对分类产生的贡献更大, 则会分配较大的权重(Transformer通过Q,K,V三个矩阵实现了Attention,并且可以兼顾较长的上下文).
>
> 对句子的编码目的是为了让模型记住token的语义(即将一个单词映射到高维向量, 并且考虑了位置等信息). 传统的LSTM只能通过长短期记忆的方法来捕捉token之间的关系, 容易导致梯度消失或记忆模糊问题, 而Transformer中, 任意两两token之间都有显式的连接, 避免了长距离依赖性问题.

## 经典的Pre-training方法

### Masked Language Modeling(MLM)

以word2vec, GloVe为代表的词向量模型, 主要以词袋(N-Gram)为基础. 如在word2vec的CBOW方法中, 随机选取一个固定长度的词袋区间, 然后挖掉中间部分的词, 让模型(简单深度神经网络)预测该位置的词

![img](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/d2f37671b6cc4c6fb4f88021e41d1a75.png)

MLM采用了N-Gram的方法, 但N-Gram喂入的是被截断后的文本, 而MLM是完整的文本, 因此MLM更能保留原始的语义

![img](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/5d2a6b9a27ef4176a658a1b7a0e0d8a0.png)

MLM是一种自监督的训练方法, 其先从大规模的无监督语料上通过固定的替换策略获得自监督语料, 设计预训练目标来训练模型, 流程如下:

​	替换策略: 在所有语料中, 随机抽取15%的文本. 被选中的文本中, 有80%随机挑选一个token并替换为[mask], 10%随机挑选一个token替换为其他token, 10%保持不变

​	训练目标: 当模型遇到[mask] token时, 根据学习得到的上下文语义去预测该位置可能的词.

​					因此, 训练的目标是对整个词表上的分类任务, 可以使用交叉信息熵作为目标函数

现有诸多针对MLM的改进版本, 作者挑选了两个经典的改进进行介绍:

​	Whole World Masking(WWM): BERT的MLM基于word piece进行随机替换, 而WWM表示被mask的必须是一个完整的单词

​	Entity Mention Replacement(EMR): 通常是在知识增强的预训练场景中, 对文本中的整个实体进行mask,而不是单一的token或字符

### Next Sentence Prediction(NSP)

在BERT原文中, 添加了NSP任务, 主要目标为给定两个句子, 判断他们之间的关系, 属于一种自然语言推理(NLI)任务, 在NSP中存在三种关系:

​	entailment: 蕴含关系, 认为位置相邻的两个句子属于entailment

​	contradiction: 矛盾关系, 认为两个句子不存在任何前后关系

​	Neutral: 中性关系, 认为当前两个句子可能来自同一篇文章, 但不属于isNext(entailment)关系

![image-20240605010243206](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/image-20240605010243206.png)

通过这种方式, 来预测这两个句子属于上述三种关系的哪种关系(感觉确实没多大用)

在之后的模型中, 由于发现NSP对实验效果并没有太多正向效果, 因此均删除了NSP的任务

# Fine-Tuning的场景

获得了预训练模型后, 在面对具体的下游任务时, 需要进行微调. 通常微调的任务目标取决于下游任务的性质. 简单列举几种NLP有关的下游任务:

## 单句分类(Single-text Classification)

 常见的单句分类任务有短文本分类, 长文本分类, 意图识别, 情感分析, 关系抽取等. 给定一个文本, 喂入多层Transformer模型, 在获得最后一层的隐状态向量后, 再输入到新添加的分类器MLP中进行分类. 在Fine-Tuning阶段, 通过交叉信息熵损失函数训练分类器(这段话的意思就是, 原本Transformer模型的最后一层对应的是其他东西(以GPT为例, 返回的就是预测的下一个词的向量, 通过矩阵乘法可以计算出预测出的向量与token库中token的相似度, 从而得出预测概率)).

> 短/长文本分类: 直接对句子进行归类, 如新闻归类, 主题分类 ,场景识别等
>
> 意图识别: 根据给定的问句判断其意图, 常用于检索式问答, 多轮对话, 知识图谱问答等(即判断对方问这个问题是想表达什么意思, 如"你吃了吗"是问候语还是真的在问你吃没吃)
>
> 情感分析: 对评论类型的文本进行情感取向分类或打分(感觉属于短文本分类中的一种?但由于比较常见就单拿出来了?)
>
> 关系抽取: 给定两个实体及对应的一个描述类句子, 判断这两个实体的关系类型

## 句子匹配/成对分类(Sentence-pair Classification)

常见的匹配类型任务有语义推理, 语义蕴含, 文本匹配与检索等. 给定两个文本, 用于判断其是否存在匹配关系. 实现方式为: 将两个文本拼接后喂入模型中, 训练策略与Single-text Classification一样(就和上面提到的NSP一样)

> 语义推理/蕴含: entailment,contradiction, neutral三种推理关系
>
> 文本匹配与检索: 输入一个文本, 并从数据库中检索与之高相似度匹配的其他句子(LLM的RAG增强中的向量库检索的原理应该就是这个)

## 区间预测(Span Text Prediction)

常见的任务类型有抽取式阅读理解, 实体抽取, 抽取式摘要等. 给定一个passage和query, 根据query寻找passage中可靠的子序列作为预测答案. 通常该类任务需要模型预测区间的起止位置, 因此在Transformer头部添加两个分类器以预测两个位置(也就是说, 以这两个分类器给出的结果作为action, 然后使用reward函数(或称为loss函数)进行评价,进而训练模型, 模型目标就是尽可能准确的得到这两个值)(不过, 是不是会有不止一个区间, 如在实体抽取中, 应返回所有的实体?)

> 抽取式阅读理解: 给定query和passage, 寻找passage中的一个文本区间作为答案
>
> 实体抽取: 对一段文本中寻找所有可能的实体
>
> 抽取式摘要: 给定一个长文本段落, 寻找一个或多个区间作为该段落的摘要

## 字符分类(Single-token Classification)

此类涵盖序列标注, 完形填空, 拼写检测等任务. 获得给定文本的隐状态向量后, 喂入MLP中, 获得每个token对应的预测结果, 并采用交叉熵进行训练

> 序列标注: 对给定的文本的每个token进行标注, 通常有词性标注, 槽位填充, 句法分析, 实体识别等(即标注语法含义和语义含义)
>
> 完形填空: 与MLM一致, 预测给定文本中空位处可能的词
>
> 拼写检测: 对给定的文本中寻找在语法或语义上的错误拼写, 并给出正确答案

## 文本生成(Text Generation)

文本生成任务常用于生成式摘要, 机器翻译, 问答等. 通常选择单向的预训练语言模型实现文本的自回归生成.

> 生成式摘要: 对给定的文本, 以生成方式获得摘要(即非摘抄原文, 而是总结)
>
> 机器翻译: 给定原始语言的文本, 来生成目标语言的翻译句子(谁能想到这是Transformer最初的用途)
>
> 问答: 给定query, 生成答案(通过特定的截断标识符来区分user的问题和LLM生成的回答)

这五类任务在fine-tuning阶段几乎都涉及在模型头部引入新参数的情况(因为目标不同, 或者说, 要生成的东西不同), 且都存在小样本场景过拟合的问题

# Prompt-Tuning

## Prompt-Tuning的定义

Prompt的目的是将Fine-tuning的下游任务目标转换为Pre-training的任务

以二分类的情感分析为例, 给定一个句子[CLS] I like the Disney films very much. [SEP]. 传统的FIne-tuning方法是将其通过预训练模型的Transformer获得[CLS]表征后再喂入新增加的MLP分类器进行二分类, 也就是说, 去识别预训练模型Transformer原头部返回的含义, 这个含义可能包含很多, 其中就包括了这个句子是积极的还是消极的这一含义, 而通过新增加的MLP分类器对其进行感知, 通过数据训练优化权重.

而Prompt-Tuning执行如下步骤:

1. 构建模板(Template Construction): 通过人工定义, 自动搜索, 文本生成等方法, 生成与给定句子相关的一个含有[MASK]标记的模板. 例如"It was [MASK]." 并拼接到原始文本中, 获得Prompt-Tuning的输入: [CLS] I like the Disney films very much. [SEP]. It was [MASK].[SEP]. 将其直接喂入(作为输入)BERT模型中, 并复用预训练好的MLM分类器, 即可直接得到[MASK]预测的各个token的概率分布
2. 标签词映射(Label Word Verbalizer): 因为[MASK]部分我们只对部分词感兴趣(如在二分类的情感分析中, 可能只对negative和positive两个词感兴趣), 因此需要建立一个映射关系, 例如如果[MASK]预测的词是"great", 则认为是positive类, 如果是"terrible", 则认为是negative类
3. 根据Verbalizer, 可以获得制定Label word的预测概率分布, 并采用交叉信息熵进行训练. 此时因为只对预训练好的MLM head进行微调, 所以避免了过拟合问题(此时的微调就如, 模型回答是可能积极也可能消极, 而标签是积极, 因此模型会调整权重, 使得其回答更积极一些, 但其回答的本质还是许多token的概率分布, 只是其中的token的词性更积极一些)

> 因为不同的句子可能有不同的Template和Label word, 因此, 如何最大化的寻找当前任务更合适的Template和Label word是Prompt-Tuning非常重要的挑战
>
>  
>
> 我们可以将引入的模板和标签词理解为一种数据增强, 通过添加提示的方式引入先验知识

## Prompt-Tuning的发展

作者应该是主要研究NLP的文本分类方向, 对于文本分类方向的Prompt-Tuning的发展和涉及到的方法非常了解, 有感兴趣的请去阅读原文章. 本人只是粗浅的阅读和记录了一下

### Prompt-Tuning的鼻祖 - GPT3与PET

GPT3提出了in-context Learning和demonstrate Learning

PET启发于文本分类任务, 试图将所有的分类任务转换为与MLM一致的完形填空. PET详细设计了Prompt-Tuning的重要组件-Pattern-Verbalizer-Pair(PVP), 并描述了Prompt-Tuning如何实现Few-shot/Zero-shot Learning.

​	Pattern(Template): 记作T, 其为额外带有[mask]标记的短文本, 通常一个样本只有一个Pattern

​	Verbalizer: 记作V, 即标签词的映射, 对于具体的分类任务需要选择指定的标签词, 因而有不同的映射. V的构建取决于对应的Pattern(T)

​	上述两个组件被称为PVP

### 如何挑选合适的Pattern

人工构建

启发式法: 通过规则, 启发式搜索等方法构建合适的模板

生成: 根据给定的任务训练数据(通常是小样本场景), 生成出合适的模板

词向量微调: 显式地定义离散字符的模板, 但在训练时, 这些模板字符的词向量参与梯度下降, 初始定义的离散字符用于作为向量的初始化(暂时没看懂)

伪标记: 不显式地定义离散的模板, 而是将模板作为可训练的参数

前面三种被称为离散的模板构建法, 旨在直接与原始文本拼接显式离散的字符, 且在训练中始终保持不变(指这些离散字符的词向量在训练过程中保持固定), 离散法不需要引入任何参数(就和你在一句话之前加上了一句话一样, 对LLM来说是一样的)

后两种被称为连续的模板构建法, 其旨在让模型在训练过程中根据具体的上下文语义和任务目标对模板参数进行连续可调. 这套方案的动机是认为离散不变的模板无法参与模型的训练环节, 容易陷入局部最优, 而如果将模板变为可训练的参数, 那么不同的样本都可以在连续的向量空间中寻找合适的伪标记, 同时也增加了模型的泛化能力. 因此, 连续法需要引入少量参数并让模型在训练时进行参数更新.(还是没看懂)

#### 启发式法构建模板

采用规则, 正则化模板的方法自动构建出相应的Pattern, 或者直接通过启发式搜索的方法获得Pattern. 这类方法在程序设计时只需要编写规则和少量模板即可快速获得Pattern

#### 生成法构建模板

基于规则的方法构建模板虽然简单, 但这些模板都是"一个模子刻出来的", 在语义上很难做到与句子贴合. 因此一种策略就是直接让模型来生成合适的模板.

LM-BFF提出了基于生成的方法构建Pattern, 而给定相应的Pattern后, 再通过搜索的方法得到相应的Verbalizer

![img](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/e0dc87ed8e114c4abf4986a243477aed.png)

首先定义一个Template的母版(只含一个mask), 将这些母版与原始文本拼接后喂入T5模型(自回归式生成模型)后在<X>和<Y>占位符部分生成相应的字符, 最终形成对应的Template, 然后再基于生成的Template和Label word训练.

#### 连续提示模板

不论是启发式方法还是通过生成的方法, 都需要为每一个任务单独设计对应的模板, 因为这些模板都是可读的离散的token, 导致修改模板会使测试结果差异很明显, 因此, 离散的模板存在方差大, 不稳定等问题. 为了避免这些问题, "连续提示"被提出, 其将模板转换为可以进行优化的连续向量, 换句话说, 我们不需要显式地制定这些模板中各个token具体是什么, 而只需要在语义空间中表示一个向量即可, 这样, 不同的任务, 数据可以自适应地在语义空间中寻找若干合适的向量, 来代表模板中的每一个词, 相较于显式的token, 这类token称为伪标记(语言模型首先将输入分为若干token, 然后再将token转换为词向量,词向量的几何表示就是在语义空间中的一个向量, 而伪标记相当于省去了token转换词向量这一步, 因为相较于词向量所能表示的含义, token只是占很少一部分, 因此当使用伪标记作为模板时, 会使表达的含义更加准确, 但也会牺牲可读性)

##### Prompt Tuning

该方法率先提出了伪标记和连续提示的概念, 以让模型能动态的对模板在语义空间内进行调整, 使得模板是可约的(differentiate)

> 给定n个tokens, 记作x1.....xn, 一个预训练模型对应的embedding table(即将token转换为词向量的表, token与词向量一一对应), 通过这个table可以将每个token表示为一个embedding. 连续模板中的每个伪标记v~i~可以视为参数, 也可以视为一个token(未必可以找到一个token与之对应, 但其表意是一个token的表意), 可以通过建立一个embedding table(与上述的embedding table不是同一个矩阵, 对象不同,作用相同)来规定伪标记的总数量. 将伪标记喂入MLP获得新的表征. 最后, 对于预训练模型的输入同时包含v~i~和x

每个伪标记的初始化可以有下列几种情况:

​	随机初始化: 即随机初始化一个面向所有伪标记的embedding table ,可采用正态分布或均匀分布等

​	每个token使用预训练模型已有的embedding table进行初始化, 此时, 每个伪标记先随机制定为词表中的一个词, 并取对应词的embedding作为这个伪标记的初始化

​	在分类任务中, 使用Label word对应的embedding作为初始化, 可以有效限制模型输出的是预设的输出类对应的word(因为即使后续进行训练, 使得词向量有所偏移, 其核心含义也不会有太大改变, 如情感分析中, 则在table中加入了negative, positive两词, 在训练中, 又训练出了询问表达的情感是这两个词中哪一个的词向量(组), 就可以组成一个人类认为合适的模板)

在训练过程中, 每个伪标记及其对应的MLP参数都可以得到训练, 对于不同的输入句子x, 伪标记对应的embedding各不相同(即模板各不相同)

##### P-tuning

![image-20240605170215974](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/image-20240605170215974.png)

累了()

## Prompt-Tuning的本质

最初的Prompt Tuning是旨在设计Template和Verbalize来解决基于预训练模型的小样本文本分类. 然而事实上, NLP领域涉及到很多除了分类以外其他大量的复杂任务(如我之前参加的那个电力大模型的项目, 也可以理解为文本分类), 如抽取, 问答, 生成, 翻译等, 并不是简单的PVP就可以解决的, 因而, 我们需要提炼出Prompt Tuning的本质, 将Prompt Tuning 升华到一种更加通用的范式上.

根据作者对Prompt-Tuning两年多的研究经验, 总结了三个关于Prompt的本质, 如下:

​	Prompt的本质是一种对任务的指令

​	Prompt的本质是对预训练任务的复用(即基于预训练模型返回结果的微调?)

​	Prompt的本质是一种参数有效性学习

### Prompt是一种针对任务的指令

Prompt本质上是对下游任务的指令, 可以作为一种信息增强. 简单来说, 就是告诉模型需要做什么任务, 输出什么内容. 上文所提及的离散或连续的模板, 本质上就是一种对任务的提示. 当数据集不同时, 我们期望模型能自适应地选择不同的模板, 这也相当于说不同的任务会有不同的对应提示信息. 我们应设计一个能够突出该任务特性的模板(如对电影评论进行二分类时, 最简单的模板是[x]. It was [mask], 但这很莫名其妙, 容易让人难以理解, 而改为The movie review is [x]. It was [mask]. 就容易理解很多)然后根据mask位置输出的结果通过Verbalizer映射到具体的标签上. 这类具备任务特性的模板可以称之为指令(Instruction).

![image-20240605170735272](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/image-20240605170735272.png)

换言之, 把LLM当成一个人, 真正的智能体(当然, 在理解了Transformer之后, 也许这并没有错, 一个人所真正接触到的信息, 与当前最大的LLM所接收到的信息可能也是有过之而无不及, 无他, 思想耳, LLM只是被灌输进了很多知识, 并将其粗浅的以高维空间的形式存到了它的数据中, 但它并没有提出疑问的机会. 不禁想象是否有人会从零教导一个LLM, 其结果又会是怎样. LLM与LLM之间的语言又是怎样的(我是指, 脱离了token转化为词向量这一步)什么时候会出现让LLM主动说话的技术呢), 指令的任务就是, 尽可能清晰的表达你的目标, 表达你的需求和要求.

在真实使用过程中, 预训练模型很难"理解"这些指令, 主要有以下几个原因:

​	预训练模型不够大: 有工作发现, 小模型在运行Prompt Tuning到时候会比Fine-tuning效果差, 因为小模型很容易收到模板的影响(我的理解是, 因为小模型没有对世界建立起足够完善的认知, 在参数层面上, 如词向量维度不够, 导致无法表达更多的信息, Attention机制层数不多, 无法关注到所有的上下文关系等).

​	缺乏指令相关的训练: 这些小模型在预训练阶段没有专门学习如何理解一些特殊的指令. 换而言之, 并不能区分提示和所要回答的问题, 类似于题干和选项.

### 复用预训练目标-实现基于Prompt的统一范式

目前绝大多数的双向预训练语言模型都包含MLM, 单向预训练语言模型都包含Autogressive Language Modeling(ALM), 这些任务是预训练目标, 其本质是预测被mask的位置的词. 之所以设计Template和指令, 就是希望在下游任务时能够复用这些预训练的目标, 避免引入新的参数而导致过拟合.

### Prompt的本质是参数有效性学习

> 参数有效性学习的背景: 在一般的计算资源条件下, 大规模的模型很难进行全量微调, 因为所有参数都需要计算梯度并进行更新, 消耗时间和空间资源. 为了解决这个问题, 参数有效性学习被提出, 其旨在确保模型效果不受太大影响的条件下, 尽可能地提高训练的时间和空间效率.
>
> 参数有效性训练: 在参数有效性学习的过程中, 大模型只需要指定或额外添加少量的可训练参数, 而其余的参数全部冻结(不予更新), 这样就可以在大大提高模型的训练效率的同时, 确保指标能够得到优化.

常见经典的参数有效性学习: Adapter-Tuning, Prefix-Tuning, BitFit

#### Adapter-Tuning

该方法固定Transformer的全部参数, 然后再Transformer的每一个block里嵌入一些新初始化的Adapter Network

![img](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/41f2ac65394c4e819629f1debeaf6a96.png)

Adapter位于Feed-Forward layer之后, 残差连接之前. Adapter本质上就是两层MLP, 分别负责将Transformer的表征降维和升维. 基于Adapter的方法, 只需要添加不到5%的可训练参数, 即可几乎达到全参数训练的效果. 在真实场景应用时, 不同的任务我们不需要重新对整个预训练模型进行微调, 只需要保存Adapter即可.

#### Prefix-Tuning

其收到Prompt-Tuning的启发, Prompt-Tuning将整个预训练模型参数全部固定, 只对Template对应的少量参数(如连续模板中的Prompt Encoder, 伪标记对应的embedding等)进行训练. 在Prefix-Tuning中, 则是除了对输入层添加模板外, 还对Transformer的每一层添加"模板".

![img](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/b55d1a89cff34c4181c44103a47e86bf.png)

Transformer的参数完全固定, 只需要对Prefix部分进行训练即可, 对于不同的任务训练不同的Prefix, 在实际使用时, 挑选任务相关的Prefix和Transformer进行组装, 即可实现可插拔式的应用.

Prefix-Tuning应用于Transformer层的key和value

#### P-tuning V2

与Prefix-Tuning相似, 不同之处在于Prefix-Tuning面向文本生成领域,而p-tuning V2面向自然语言理解. 本质上完全相同.

![在这里插入图片描述](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/4240913533d3411a91a099396e1a89da.png)

左图部分是基于连续提示的Prompt-Tuning只有输入层对应模板部分的Embedding和MLP参数是可训练的, 右图部分表示的事Prefix-Tuning(P-tuning V2), Transformer的每一层的前缀部分也是可以训练的, 可以抽象的任务是在每一层添加了连续的模板. 但实际上, Prefix-Tuning并不是真正的在每一层添加模板, 而是通过Huggingface框架内置的past_key_value参数控制. 本质与adapter类似, 是在Transformer内部对Key和Value插入可训练的两个MLP.

#### LoRA(Low-Rank Adaptation)

参考: [LORA详解（史上最全）_lora模型-CSDN博客](https://blog.csdn.net/qq_41475067/article/details/138155486)

​		 [LoRA的原理简介_lora机制-CSDN博客](https://blog.csdn.net/stingfire/article/details/138315770)

一种用于微调大型语言模型的低秩适应技术. 它最初应用于NLP领域, 特别是用于微调GPT3等模型. LoRA通过仅训练低秩矩阵, 然后将这些参数注入到原始模型中, 从而实现对模型的微调. 这种方法不仅减少了计算需求, 而且使得训练资源比全量微调小得多.

> *在Stable Diffusion模型的应用中, LoRA被用作一种插件, 允许用户在不修改SD模型的情况下, 利用少量数据训练出具有特定画风, IP或人物特征的模型.*

在使用时, 将LoRA模型与预训练模型结合使用, 通过调整LoRA的权重可以改变输出.

LoRA模型的优点包括: 训练速度快, 计算需求小, 训练权重低

LoRA技术通过将权重矩阵(W)分解成低秩矩阵的乘积(与Transformer中的V矩阵相似?),降低了参数数目, 进而达到了减少硬件资源, 加速微调进程的目的.

LoRA在保留基座模型全部参数的同时, 拆分出权重矩阵进行矩阵分解并更新, 通过调整训练中由低秩矩阵乘积表示的更新矩阵来在减少存储空间的同时保留了模型的质量和微调速度.

![img](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/0d8ae9facf45486999d9b8e68d7a6535.png)

对于一个预训练好的基座模型, 保留其原有的权重矩阵W不变, 仅微调训练更新部分, 且这个更新的权重矩阵被分解为A和B两个低秩矩阵, 其中A矩阵初始化为高斯分布矩阵, B矩阵初始化为全0矩阵.

![image-20240605205117643](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/image-20240605205117643.png)

在实际场景应用时, 会引入两个超参数: α和r, 二者之比α/r对△W(更新的权重矩阵)进行缩放, 控制△W的更新步长

LoRA作者发现, 仅对W~q~(查询的权重矩阵,在Transformer中会有讲解)进行分解更新的效果不够好, 但对全部四个权重矩阵进行更新并没有大幅提升. 对W~q~和W~v~进行分解更新相对而言效果最好. r一般取4或8.

#### UniPELT

参考: [UniPELT: A Unified Framework for Parameter-Efficient Language Model Tuning - 郑之杰的个人网站 (0809zheng.github.io)](https://0809zheng.github.io/2023/02/14/unipelt.html)

相关工作对Adapter, Prefix-Tuning, LoRA等参数有效性学习进行了集成, 因为这些参数有效性学习方法本质上都是插入少量的新的参数, 这些新的参数可以对预训练模型起到提示作用, 只不过并不是以人类可读的离散的模板形式体现而已.

UniPELT将LoRA,Prefix Tuning, Adapter结合在一起, 通过门控机制学习激活最适合当前数据和任务的方法.

![img](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/f1211d0e55f64e7b92714d3692efa290.png)

对于每个模块, 通过线性层实现门控, 通过G~p~参数控制Prefix-Tuning方法的开关, G~L~控制LoRA方法的开关, G~A~控制Adapter方法的开关. 可训练参数包括LoRA矩阵, Prefix-tuning参数, Adapter参数和门控函数权重(图中蓝颜色的参数).

UniPELT方法始终优于常规的全量微调以及它在不同设置下包含的子模块，通常超过在每个任务中单独使用每个子模块的最佳性能的上限；并且通过研究结果表明，多种 UniPELT 方法的混合可能对模型有效性和鲁棒性都有好处。

#### BitFit

只需要指定神经网络中的偏向(Bias)为可训练参数即可. BitFit的参数量只有不到2%, 但实验效果接近全量参数.

假设输入层用向量X表示，则隐藏层的输出就是 f (W1X+b1)，W1是权重（也叫连接系数），b1是偏置，函数f 可以是常用的sigmoid函数或者tanh函数

# 面向超大规模模型的Prompt-Tuning

对于超过10亿参数量的模型来说, Prompt-Tuning所带来的增益远远高于标准的全量的Fine-tuning. 小样本甚至零样本的性能也能够极大地被激发出来, 这得益于这些模型的参数量足够大, 训练过程中使用了足够多的语料, 同时设计的预训练任务足够有效. 对于GPT3, 只需要设计合适的模板或指令即可实现免参数训练的零样本学习.(换而言之, 对于一个足够成熟, 并且掌握了相关背景知识的人, 你只需要给他一个提示, 或者进行一个培训就可以让这个人做一些工作, 而没必要从头到尾重新塑造一下这个人)

几个面向超大规模的Prompt-Tuning方法:

​	上下文学习 In-Context Learning(ICL): 直接挑选少量的训练样本作为该任务的提示

​	指令学习	Instruction-Tuning: 构建任务指令集, 促使模型根据任务指令作出反馈

​	思维链		Chain-of-Thought(CoT): 给予能够激发模型推理和解释能力的信息, 通过线性链式的模式指导模型生成合理的结果.

## In-Context Learning(上下文学习)

旨在从训练集中挑选少量的标注样本, 设计任务相关的指令形成提示模板, 用于指导测试样本生成相应的结果.

![image-20240605202343016](https://tuchuang-yzs.oss-cn-beijing.aliyuncs.com/image-20240605202343016.png)

ICT在预测过程中, 存在方差大, 不稳定的问题.

### 样本的Input-Output Mapping的正确性是否对ICL有影响

结果:

​	使用Demonstration比不使用的效果好

​	random Label对模型性能的破坏不是很大, 说明ICL更多的是去学习Task-specific的Format, 而不是Input-Output Mapping.

> MetaICL是一种通过任务统一范式并使用元学习进行训练的方法, 其重点增加了多任务的训练来改进ICL在下游零样本推理时的泛化性能.





太多了, 之后再整理()

















































