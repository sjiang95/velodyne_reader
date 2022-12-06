# velodyne reader

This repo provides python scripts to read data stream directly from velodyne lidars.

## Prerequisite

```shell
conda create -n lidar --file requirements.txt -c conda-forge
conda activate lidar
pip install velodyne_decoder
```

Or, if you prefer `pip`

```shell
pip install -r requirements.txt
pip install velodyne_decoder
```

## Applicability

Generally, the supported model is decided by [valgur/velodyne_decoder](https://github.com/valgur/velodyne_decoder#configuration). You can get the supported device list by

```shell
$ python -c "import velodyne_decoder as vd;print(vd.Config.SUPPORTED_MODELS)"
['HDL-32E', 'HDL-64E', 'HDL-64E_S2', 'HDL-64E_S3', 'VLP-16', 'VLP-32C', 'Alpha Prime']
```

## Usage

### Independently

Take `VLP-16` for example, the following command runs a 60-second demo, and write the point cloud to `out/<date>/<model>_<rpm><returnMode>_<UTCtimestamp>.pcap`, together with cooresponding `.log` file. The `.pcap` file can be visualized directly by [VeloView](https://www.paraview.org/VeloView/).

```shell
python vldreader.py --model VLP-16 --ip-lidar <ip_addr_lidar> --dataport <data_port_lidar> --rpm <rpm>
```

If you did not tune the lidar's ip address and its data port, simply run

```shell
python vldreader.py --model VLP-16 --rpm 1200
```

### As a component

Of course, you can also use this script as a module in your project.

```python
from /path/to/vldreader.py import ld

myld=ld(model='VLP-16', lidarip='192.168.1.201', dataPort=2368, rpm=600, returnMode='dual')

# launch the device
myld.launch()

# read stream data
for onepkt in myld.read_live_data():
    print(onepkt)

# stop the device
myld.stop()
```
