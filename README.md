# コードの手順(on Docker)
1. zense_grpc_module直下で"make build"を実行し、dockerコンテナを作成する
2. make runでコンテナ内部に入り、setup.shを実行する(コードのビルドやgRPC依存ファイルの生成を行う)

# 実行例
1. (初回のみ)センサをPCに接続しLEDが点灯することを確認した後、センサの設定ファイルを生成する
```
python scripts/utils/generate_zense_toml.py
cd {zense_grpc_moduleディレクトリのパス}
```
2. センサモジュールを起動する
```
cd build && ./pico_zense_simple_publisher
```
3. (別窓で)ビューワクライアントを起動する
```
cd scripts && python rgbd_viewer_client.py
```
