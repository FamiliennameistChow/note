# linux常见问题

### 修复win下硬盘不识别问题
```
sudo ntfsfix /dev/sda2
```

### 查看安装的程序包

```
dpkg --get-selections | grep firefox
```