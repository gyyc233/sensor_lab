
因为电脑2坤年了，加上win11的bug频出，导致我虚拟机经常异常关机，使得git仓库有时候会数据损坏，在pull期间出现以下错误`error object file is empty , The remote end hung up unexpectedly`

做之前必须备份你的仓库

```bash
# 1. 删除任何空文件
# find .git/objects/ -type f -empty -delete
find .git/objects/ -type f -empty | xargs rm

# 2. 下载上一步删除的对象文件
git fetch -p

# 3. 全面储存对象检查
git fsck --full

# 4. try pull
git pull
```
