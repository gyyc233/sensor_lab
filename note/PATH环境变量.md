- [环境变量](#环境变量)
  - [环境变量配置](#环境变量配置)
    - [永久设置环境变量](#永久设置环境变量)
  - [PATH变量](#path变量)
  - [查看环境变量](#查看环境变量)

# 环境变量

环境变量是操作系统提供给应用程序访问的简单 key/value 字符串

- 我们从命令行想要运行一个程序的时候，待运行的程序往往不是在当前目录, PATH变量就是用于保存可以搜索的目录路径，如果待运行的程序不在当前目录，操作系统便可以去依次搜索PATH变量变量中记录的目录，如果在这些目录中找到待运行的程序，操作系统便可以运行

## 环境变量配置

终端中，可以使用export命令临时设置环境变量

```shell
export TEST_HOME=/usr/local/lib/test_home
```

### 永久设置环境变量

- `/etc/profile`：对所有用户生效。
- `~/.profile`：对当前用户生效。
- `~/.bashrc`：对当前用户生效。
- `/etc/environment`：对所有用户生效

## PATH变量

PATH变量是环境变量中最重要的一个，定义了系统在执行命令时搜索可执行文件的目录路径

例如，要添加一个新的路径到PATH变量：

```shell
export PATH=$PATH:/new/path
```

一次性添加多个路径

```shell
export PATH=$PATH:/new/path1:/new/path2
```

## 查看环境变量

`printenv` or `env`


查看单个环境变量 `echo %TEST_HOME`
