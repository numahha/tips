# NIC

買ったPCにUbuntu 18を入れてネット回線に繋がらなかったので。

まず、ドライバを特定する。

```
$ lspci | grep Ethernet
```

結果、Intel Corporation Ethernet Connection (7) I219-V (rev 10)というやつらしい。
[参考](https://qiita.com/nitomath/items/40e5ee62a1c3c9dca78c)に沿って、
[このページ](https://downloadcenter.intel.com/download/15817)からドライバ(e1000e-3.4.2.4.tar.gz)を入手する。

これを適当な場所で解凍し、`e1000e-3.4.2.4/src/nvm.c`の`e1000e_validate_nvm_checksum_generic`を、以下のように弄る。

```
s32 e1000e_validate_nvm_checksum_generic(struct e1000_hw *hw)
{
        return 0;
}
```

[参考](https://qiita.com/nitomath/items/40e5ee62a1c3c9dca78c)に沿って、セキュアブートが無効化されているかどうかを確認する。

これをインストールすることになるのだが、ubuntu 18をインストールした時点では`make`コマンドが入っていない。
`make`コマンドをインストールするために、一時的にUSBテザリングを使ってネット回線を確保した上で、次のコマンドを実行する。
```
$ sudo apt-get update
$ sudo apt-get install build-essential
```
これで、`make`コマンドが使えるようになったので、USBテザリングの利用を終了してOK。

`e1000e-3.4.2.4/src/`以下で、次のように実行する。

```
$ sudo make install
$ sudo modprobe -r e1000e
$ sudo modprobe e1000e
```

`$ sudo make install`の時に、「catman モードで /var/cache/man/cat7/e1000e.7.gz に書き込みできません」と怒られるけど、無視して進めればOK。
とりあえず、これでネット回線につながることになる。

再起動時にこのドライバを読み込むようにするために、以下を実行する（[参考2](https://forums.ubuntulinux.jp/viewtopic.php?pid=119749)）。
```
$ sudo update-initramfs -u
```

