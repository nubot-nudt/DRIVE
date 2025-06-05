import os

# 指定源文件和目标文件的路径
source_folder = 'my_data/rgb/'  # 源文件所在的文件夹
target_folder = 'data/rgb/'  # 目标文件所在的文件夹

# 确保目标文件夹存在
if not os.path.exists(target_folder):
    os.makedirs(target_folder)

# 遍历文件名范围
for i in range(10001):
    # 构造源文件和目标文件的完整路径
    source_file = os.path.join(source_folder, "{}.png".format(i))  # 假设扩展名为.jpg
    target_file = os.path.join(target_folder, "{}.png".format(i + 10000))  # 目标文件也使用相同的扩展名

    # 检查源文件是否存在
    if os.path.exists(source_file):
        with open(source_file, 'rb') as src, open(target_file, 'wb') as tgt:
            # 读取源文件内容并写入目标文件
            content = src.read()
            tgt.write(content)