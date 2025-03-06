# 添加模块
1. 首先修改_includes中的header.html
2. 然后修改_includes中的nav.html
3. 修改_sass/pages/home-blog-projects.sass(注意, 要全部小写)
4. 在_config.yml添加相应标签
5. 创建对应名称的html
6. congratulation, 成功建立

# 添加博客报错
## "{{"问题
查找是否存在"{{" 如果存在, 在代码块上下分别添加"{% raw %}"和"{% endraw %}"(不要加上""), 即可解决问题

# 博客照片不显示问题
大概率是因为用的国内的网页做图床, 导致github无法访问, 所以无法显示, 我用gitee做图床就无法显示, 但是用阿里云做图床就可以显示
