import pandas as pd

# 读取CSV文件
file_path = './out.csv'  # 请将 'your_file.csv' 替换为你的实际文件路径
df = pd.read_csv(file_path)

file_path = './half.csv'  # 请将 'your_file.csv' 替换为你的实际文件路径
df2 = pd.read_csv(file_path, header=None, index_col=False)
# df2 = df2.iloc[0:, :]

# 删除第1行数据
df = df.iloc[1:, :]

# 获取第132行以后的数据
data_to_copy = pd.DataFrame([100] * 10)

# 复制数据10次
# copied_data = pd.concat(, ignore_index=True)
print(df2[0])

# data_to_copy.columns = df2[0].columns


# 将复制后的数据拼接到原始数据框中
for i in range(12):
    if i == 2 or i==4 or i==8 or i==10:
        df2[i] = pd.concat([data_to_copy, df2[i]],axis=0, ignore_index=True)
    else:
        print(i)
        df2[i] = pd.concat([df2[i],data_to_copy],axis=0, ignore_index=True)
        

print(df2)
