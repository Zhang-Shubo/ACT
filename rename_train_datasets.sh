#!/bin/bash
set -ex


# 设置目标文件夹

target_dir="/Users/zhangdavid/code/ACT/data/task1"



# 校验文件夹是否存在

if [ ! -d "$target_dir" ]; then

    echo "目标文件夹不存在: $target_dir"

    exit 1

fi



# 进入目标文件夹

cd "$target_dir" || exit



# 获取所有 .hdf5 文件并以自然排序

files=($(ls *.hdf5 | sort -V))



# 重命名文件为连续的名称，从0开始

counter=0

for file in "${files[@]}"; do

    new_name="episode_$counter.hdf5"

    mv "$file" "$new_name"

    echo "Renamed: $file -> $new_name"

    ((counter++))

done



echo "重命名完成！"