# 初期設定メモ

* makeを入れる
sudo apt-get update
sudo apt-get install build-essential

* e1000eをインストールする。

* 有線デバイスの管理

sudo gedit /etc/NetworkManager/NetworkManager.conf
>>managed = true

* pppoe
sudo apt-get install -y pppoe pppoeconf
sudo pppoeconf

プロバイダから送られてきたログインid/pwを入れる。
