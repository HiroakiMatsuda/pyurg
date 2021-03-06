pyurg
======================
pyurgは[北陽電機（株）][hokuyo]のレーザースキャナである[URGシリーズ][urg]を制御するPython Scriptです   

[hokuyo]: http://www.hokuyo-aut.co.jp/index.html
[urg]: http://www.hokuyo-aut.co.jp/02sensor/

動作確認環境
------
Python:  
2.6.6  

pySerial:  
2.6

OS:  
Windows 7 64bit / 32bit  
Ubuntu 10.04 LTS / 12.04 LTS 32bit

URG:  
UTM-30LX   


各コマンドは以下の[UTM-30LX 通信仕様書(SCIP2.0)][scip]に従って機能を提供しています  
[scip]: http://www.hokuyo-aut.co.jp/02sensor/07scanner/download/products/utm-30lx/ 
 
使い方
------
###1. pySerialをインストールする###
pyrsは[pySerial](:http://pyserial.sourceforge.net/)を使用してシリアル通信を行なっています    
pySerialがインストールされていない場合は、インストールしてから実行してください  

###2. Python ShellからURGを動かしてみる###
Python Shellからサーボモータ（ID :1）を動かしてみます  
想定しているURGはUTM-30LXです  
'COM1'の部分は自分の環境に合わせてポートを設定してください  
Ubuntuでは'/dev/ttyUSB0'のように指定します  

```python  
import pyurg  
urg = pyurg.Urg()  
urg.set_urg('COM1')  
status = urg.requese_me(0, 1080, num = 2)  
urg.check_status(status)  
for i in range(2):  
    dist, intensity, timestamp = urg.get_distance_and_intensity()  
    print timestamp, len(dist), len(intensity)  
urg.close_port()  
```   
以上のように簡単にサーボモータを制御することができます 
（サンプルでは距離と受光強度データはリストの長さしか表示していません） 
このmoduleではset_urgメソッドでポートオープンとURGの初期化を行います  
このメソッドは必ず一度使用してください  
その後URGにリクエスト出し、データを指定回数分読み込みます  
各メソッドの詳しい使い方は以下を御覧ください  

###3. メソッドの使い方###
    def open_port(port, baudrate, timeout)
シリアルポートを開きます  
 ・ `port` :  
    ポートの番号の指定。'COM1'や'/dev/ttyUSB0'のように指定  
 ・   `baudrate` :  
    ボーレートの指定。サーボと同じボーレートを指定してください   
 ・   `timeout` :  
    読み取り時のタイムアウト設定  
    None:読み取れるまで待ちます
    0:非ブロッキングモード（読み取り時にすぐ戻ります） 
    x:x [sec]待ちます  
  ・  `return` :    
    戻り値なし　　
  
    def close_port()
シリアルポートを閉じます  
  ・  `return` :    
    戻り値なし　

    def flush_outport()
シリアルポートの出力を消去します  

     def flush_inport()
シリアルポートの入力を消去します  
 
    def set_port(baudrate, timeout)
シリアルポートを再設定します  
 ・  `baudrate` :  
    ボードレートの指定。サーボと同じボードレートを指定してください   
 ・  `timeout` :  
    読み取り時のタイムアウト設定  
    None:読み取れるまで待ちます
    0:非ブロッキングモード（読み取り時にすぐ戻ります） 
    x:x [sec]待ちます  

    def set_urg(port)  
URGを使用するための初期化を行います  
URGを使用する前に必ずこのメソッドを使って初期化してください  
1.ポートオープン  
2.SCIP2.0指定  
3.URGのパラメータ取得  
4.初期値の設定  
以上の操作を行います  
 ・ `port` :  
    ポートの番号の指定。'COM1'や'/dev/ttyUSB0'のように指定  

     def reset_urg(port)  
URGとシリアルポートの初期化を行います  
通信不良や指令ミスが発生し、接続が行えなくなった場合に使用してください  
1.シリアルポートの入出力の消去    
2.ステータスのリセット  
3.ポートの切断  
4.ポートの接続  
5.シリアルポートの入出力の消去
以上の操作を行います    
・ `port` :  
    ポートの番号の指定。'COM1'や'/dev/ttyUSB0'のように指定

    def set_scip2(self)  
URGの通信プロトコルをSCIP2.Xに指定します    
 ・  `return` :文字列    
  'SCIP2.0'  

    def set_timestamp(mode, string)  
タイムスタンプ用のセンサ内の時間を合わせます  
・  `mode` :  
    時計合わせのモードを選択します    
    'TM0', 時計合わせモードに入ります      
    'TM1', 現在時刻を返します    
    'TM2', 時計合わせモードを終了し通常モードに移行します 
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :  
    TM1:timestamp  
センサ起動時から最大16777216 [ms]までのアップカウントされるタイマーが返ります  
このタイマーはスキャンの0step位置における時計値です  
このタイマーは最大値までカウントすると0にクリアされます  
   TM0, TM2:status  
コマンドのステイタスが返ります  
'00': 異常なし  
'01': コマンド制御コートが範囲外の数字  
'02': 時計合わせモード開始コマンド発行の時、すでに開始している  
'03': 時計合わせモード終了コマンド発行の時、すでに終了している  
'04': TM1コマンド発行の時、時計合わせモードでない  
  
     def turn_on_laser(string)  
レーザーを点灯させ、計測を開始します  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :status  
'00':異常なし  
'01':故障中のためレーザ制御は不可  
'02':すでにレーザの状態が指示された状態にある  

      def turn_off_laser(string)
レーザーを消灯し、計測を停止します  
request_md、requese_ms、requeset_meのキャンセルコマンドとして使用できます  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :status  
'00':異常なし    

      def reset_all_status(string)
レーザーを消灯し、起動後に設定された全てのパラメータを初期化します  
ただしSCIP2.Xの動作のままとなります  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :status  
'00':異常なし    

      def reset_status(string)
reset_all_statusからモータの回転速度と、シリアル通信の速度を除き、起動後に設定された全てのパラメータを初期化します  
SCIP2.1対応センサでのみ有効  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :status  
'00':異常なし    

      def get_version(string)  
製品情報やファームのバージョン、センサのシリアルナンバーなどを返します  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :  辞書型の製品情報  
{'MODEL': ベンダ情報,   
'PROD': 製品情報,     
'FRIM':F/Wバージョン,  
'PROT':プロトコルバージョン ,   
'SERI': シリアル番号}      

      def get_parame(string)  
計測に関する固有パラメータを返します  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :  辞書型のセンサ固有情報      
{'VEND': センサ型式情報,   
'DMIN': 最小計測可能距離 [mm],     
'DMAX':最大計測可能距離 [mm],  
'ARES':角度分解能(360°の分割数),   
'AMIN': 有効計測エリア開始ステップ番号,  
'AMAX': 有効計測エリア終了ステップ番号,  
'AFRT': センサ正面ステップ番号,  
'SCAN':標準走査角速度 [rpm],  
'RDIR': 走査回転方向}  

RDIRはTop-URGは非対応  


      def get_status(string)  
計測に関する可変パラメータを返します  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :  辞書型のセンサ可変情報      
{'MODL': センサ型式情報,   
'LASR': レーザ点灯状態,     
'SCSP':現計測速度(モータ回転速度),  
'MESM':現計測モード,   
'SBPS 現シリアル通信速度,  
'TIME': 現センサ内時計値(6桁HEX),  
'STAT':センサ状態}      

      def request_md(amin, amax, unit,  interval, num,  string)  
MDモードで計測要求を出します  
コマンド受信後に新規に取得した距離データを指定回数分返します  
MDモードでは距離データを3byteのデータとして取得します  
リクエストだけでは距離データは取得できません  
get\_distanceメソッドで距離データを別途取得する必要があります  
・  `amin` :  
    有効計測エリア開始ステップ番号を指定します  
・  `amax` :  
    有効計測エリア終了ステップ番号を指定します  
・  `unit` :  
    まとめる計測点の数を指定します  
通常の使用であれば1を指定してください  
・  `interval` :  
    間引きスキャン数を指定します（現在のpyurgでは利用できません）   
・  `num` :  
    計測回数(最大99)を指定します。0で無限回となります  
計測の途中で中断するにはturn\_off\_laser()メソッドを使用してください  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :  status 
check_statusメソッドで下記のstatusをチェックすることができます  
'00':コマンドの正常受信通知    
'01':開始ステップに数字以外の文字が混在    
'02':終了ステップに数字以外の文字が混在  
'03':まとめるステップ数に数字以外の文字が混在  
'04':終了ステップの指定数が最大設定範囲外  
'05':終了ステップの指定数が開始ステップの指定数以下  
'06':間引きスキャン数に数字以外の文字が混在  
'07':送信回数に数字以外の文字が混在  
'21~49':ステータスが示す異常可能性を検知のため通信を中断  
'50~97':レーザ異常・モータ異常などハードウェア故障  
'98':センサの正常状態が確認できたため、通信中断状態から復帰  

      def request_ms(amin, amax, unit,  interval, num,  string)  
MSモードで計測要求を出します  
MSモードでは距離データが2byte：最大4095 [mm]までの距離を返します  
それ以外はMDモード(request\_mdメソッド)と同様です  

      def request_gd(amin, amax, unit, string)  
GDモードで計測要求を出します  
コマンド受信時における最新データを返します  
GDモードでは距離データは3byteのデータとしてい取得します  
またGDモードはMDモードと異なり、計測状態である必要があります  
turn\_on\_laserメソッドで予め計測状態に設定してください  
・  `amin` :  
    有効計測エリア開始ステップ番号を指定します  
・  `amax` :  
    有効計測エリア終了ステップ番号を指定します  
・  `unit` :  
    まとめる計測点の数を指定します  
通常の使用であれば1を指定してください  
・  `string` :  
    コマンドに目印としての文字列を設定します（現在のpyurgでは利用できません）  
 ・  `return` :  status 
'00':コマンドの正常受信通知    
'01':開始ステップに数字以外の文字が混在    
'02':終了ステップに数字以外の文字が混在  
'03':まとめるステップ数に数字以外の文字が混在  
'04':終了ステップの指定数が最大設定範囲外  
'05':終了ステップの指定数が開始ステップの指定数以下  
'10':レーザー消灯中  
'50~98':レーザ異常・モータ異常などハードウェア故障  

      def request_gs(amin, amax, unit,  interval, num,  string)  
GSモードで計測要求を出します  
GSモードでは距離データが2byte：最大4095 [mm]までの距離を返します  
それ以外はMDモード(request\_mdメソッド)と同様です   

      def request_me(amin, amax, unit,  interval, num,  string)  
MEモードで計測要求を出します  
コマンド受信後に新規に得られる距離データと受光強度を指定回数分返します  
MEモードでは距離データと受光強度を3byteずつのデータとして取得します  
リクエストだけでは距離と受光強度はデータは取得できません  
get\_distance\_and\_intensityメソッドで計測データを別途取得する必要があります  
それ以外はMDモード(request\_mdメソッド)と同様です  

      def get_distance()  
リクエストモードに応じた距離データを取得します  
MD、MSモードでは指定回数分だけこのメソッドを呼んで下さい  
 ・  `return` :  dist, timestamp   
  dist:距離データのリスト  [mm]  
  timestamp:タイムスタンプ  [msec]  

      def get_distance_and_intensity()  
MEモードでのみ使用でき、距離と受光強度データを取得します  
指定回数分だけこのメソッドを呼んで下さい  
 ・  `return` :  dist, intensity, timestamp   
  dist:距離データのリスト  [mm]  
  intensity: 受光強度  
  timestamp:タイムスタンプ  [msec]  

      def check_status()  
MD、MSモードリクエスト時のステータスをチェックします  
異常があればエラーをraiseします  
 ・  `return` :  なし  

      def convert_to_xy(dist)  
距離データをXY座標に変換して返します  
・  `return` :  [[x1, y1]........[xn, yn]]  
[x, y]の座標値を格納したリストを格納した２次元のリストとして返します    

      def convert_to_x_y(dist)  
距離データをXY座標に変換して返します  
・  `return` : [x1,........xn],  [y1,........yn]  
x, yの座標値を独立したリストとして返します    

###4. URGの座標系###
URGのXY座標系は以下のように設定しています  
  
<img src="https://github.com/downloads/HiroakiMatsuda/pyurg/urg_xy_axis.png" width="300px" /> 


ライセンス
----------
Copyright &copy; 2012 Hiroaki Matsuda  
Licensed under the [Apache License, Version 2.0][Apache]  
Distributed under the [MIT License][mit].  
Dual licensed under the [MIT license][MIT] and [GPL license][GPL].  
 
[Apache]: http://www.apache.org/licenses/LICENSE-2.0
[MIT]: http://www.opensource.org/licenses/mit-license.php
[GPL]: http://www.gnu.org/licenses/gpl.html