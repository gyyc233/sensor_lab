
这几天github浏览器能上，但是无法clone, 同时虚拟机也不能登录github和git操作

试了好多种办法，结果是要更新hosts

## update git hosts

- win hosts file path `C:\Windows\System32\drivers\etc\hosts`
- ubuntu hosts file path `/etc/hosts`
- 查询github.com在全球的ip地址
  - `https://dnschecker.org/`
- update hosts file
  - tail add `20.205.243.166 github.com` etc 建议添加所在地ip（如中国）, 好像会快些
