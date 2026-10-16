# ArchiveBot ROS 2

ROS 2 ve Gazebo Sim ile gelistirilen, depo ortaminda otonom gezen ve kamera ile kontrol noktalarini denetleyen mobil robot projesi.

**Guncel / onerilen navigasyon:** `archivebot_navigation_v11.py`

## Ozellikler

- Depo icinde belirlenmis kontrol noktalarina otonom navigasyon
- 360 derece LiDAR ile engel algilama ve guvenli durus
- RGB kamera ile **RED** ve **GREEN** kontrol noktalarinin dogrulanmasi
- IMU ve tampon (contact sensor) ile ek guvenlik kontrolleri
- Gorev bitince baslangic alanina donus ve otomatik park yonlendirmesi
- Denetim sonuclarinin CSV dosyasina kaydedilmesi

> Bu proje halen gelistirme asamasindadir. Simulasyonda test edilmistir; gercek robotta kullanim icin ayri guvenlik testleri gereklidir.

## Hangi surumu calistirmaliyim?

| Dosya | Amaci | Tercih |
| --- | --- | --- |
| `scripts/archivebot_navigation_v11.py` | Tam gorev, RED/GREEN kontrolu, donus, otomatik park; 0.18 m/s duz hat hizi | **Onerilen guncel surum** |
| `scripts/archivebot_navigation_v10.py` | Tam gorev ve otomatik park; daha dusuk duz hat hizi | Onceki stabil surum |
| `scripts/archivebot_navigation_v9_full.py` | Tam gidis-donus gorevi ve kamera kontrolleri | Onceki surum |
| `scripts/archivebot_navigation_v5.py`–`v9.py` | Gelistirme / test asamalari | Yeni baslayanlar icin gerekmez |
| `scripts/archivebot_autonomy*.py` | Onceki otonomi denemeleri | Arsiv / gelistirme |
| `scripts/archivebot_vision.py` | Kameradan renk algilayip kontrol durumu yayinlar | **V11 ile beraber calistirilir** |

**Not:** V11'in baslangic logunda onceki surumden kalan "ArchiveBot V8 navigasyon hazir" yazisi gorunebilir. Calisan dosya ismi `archivebot_navigation_v11.py` ise V11'i calistiriyorsunuz.

## Gereksinimler

Proje su ortamda gelistirildi ve calistirildi:

- Ubuntu sanal makine (VirtualBox)
- **ROS 2 Lyrical** (mevcut gelistirme ortaminda `/opt/ros/lyrical`)
- Gazebo Sim (Gazebo `gz` araci ve OGRE render motoru)
- Python 3, `rclpy`, `OpenCV`, `NumPy`, `cv_bridge`
- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`, `xacro`

ROS 2 ve Gazebo'nun ilgili surumlerini once resmi kurulum rehberlerine gore kurun. ROS 2 Lyrical kurulu bir Ubuntu sisteminde eksik paketler icin ornek:

~~~bash
source /opt/ros/lyrical/setup.bash
sudo apt update
sudo apt install -y git python3-opencv python3-numpy \
  ros-lyrical-xacro ros-lyrical-cv-bridge \
  ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-interfaces
~~~

Farkli bir ROS 2 surumu kullaniliyorsa yukaridaki dagitim adini ve paket uyumlulugunu kontrol edin.

## Projeyi GitHub'dan indirme

Repository **Private** oldugu icin once proje sahibi sizi GitHub Collaborators bolumunden davet etmeli; daveti kabul ettikten sonra klonlayabilirsiniz.

~~~bash
git clone https://github.com/Batuhan88/ArchiveBot-ROS2.git ~/archivebot_ws
cd ~/archivebot_ws
~~~

> **Onemli:** Kod, CSV raporunu `~/archivebot_ws/reports/` altina yaziyor. Bu nedenle ilk kurulumda repository'yi `~/archivebot_ws` yoluna klonlamak en kolayi. O klasor zaten mevcutsa uzerine klonlamayin; farkli klasor kullanacaksaniz rapor yolunu da guncelleyin.

### Klasor yapisi

~~~text
archivebot_ws/
├── README.md
├── .gitignore
└── src/
    └── archivebot_description/
        ├── config/bridge.yaml
        ├── models/
        │   ├── archivebot.urdf.xacro
        │   └── archivebot_v9.urdf.xacro
        ├── scripts/
        │   ├── archivebot_vision.py
        │   ├── archivebot_navigation_v11.py
        │   └── ... (eski surumler)
        └── worlds/archive_room.sdf
~~~

**Paketleme notu:** `package.xml`, `CMakeLists.txt` ve tek komutluk launch dosyasi eklendi. Yeni launch henuz VM'de test edilmedi; asagidaki 6-terminal yontemi calisan referans yontemdir.



## Tek komutla baslatma (yeni, test asamasinda)

ROS 2 `ament_cmake` paket dosyalari ve `launch/archivebot.launch.py` eklendi.
Bu akis henuz gelistirme VM'inde test edilmedi; ilk denemede **navigasyon kapali** baslatin.
Eski 6-terminal yontemi halen yukarida bulunuyor ve calisan referans yontemdir.

Bir terminalde:

~~~bash
cd ~/archivebot_ws
git pull --ff-only origin main
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select archivebot_description
source install/setup.bash
ros2 launch archivebot_description archivebot.launch.py start_navigation:=false
~~~

Gazebo dunyasi, robot, iki bridge ve kamera node'u acilmalidir. Acilmazsa terminaldeki hata mesajini kaydedin.
Yeni terminalde `source /opt/ros/lyrical/setup.bash && source ~/archivebot_ws/install/setup.bash` sonrasinda:

~~~bash
ros2 topic info /scan
ros2 topic info /archivebot/ground_truth/odom
ros2 topic info /archivebot/checkpoint_status
gz model -m archivebot --pose
~~~

Bu komutlarla robot ve sensorler dogrulandıktan sonra tum launch surecini `Ctrl+C` ile kapatip yeniden calistirin:

~~~bash
ros2 launch archivebot_description archivebot.launch.py
~~~

Bu kez V11 navigasyonu da baslar. Tek komutluk launch, Gazebo'ya 20 saniyelik yukleme suresi verir; robot spawn islemi tamamlandiktan sonra V11 icin 5 saniye daha bekler. Yavas VM'lerde bu sureler yetersiz kalabilir ve ayarlanmasi gerekebilir. `archivebot_navigation_v11.py` ile robotun Xacro ve SDF dosyalarina dokunulmadi.

> Bu launch'ı, halihazirda Gazebo veya ArchiveBot node'lari calisirken baslatmayin. Cift `/cmd_vel` yayinlayicisi veya cift robot olusabilir. Terminalde `colcon: command not found` cikarsa ROS 2 ortaminda `python3-colcon-common-extensions` paketini kurun. `package.xml` dosyasindaki maintainer e-postasi ve `TODO` lisans alani proje ekibi tarafindan daha sonra doldurulmalidir.

## Calistirma (adim adim)

**Her asama icin ayri terminal acin.** Terminali kapatirsaniz o terminalde calisan islem durur. ROS 2 komutlarini calistirmadan once ilgili terminalde `source /opt/ros/lyrical/setup.bash` kullanin.

### Terminal 1 — Gazebo depo ortamini baslat

~~~bash
source /opt/ros/lyrical/setup.bash
gz sim -r -v 3 "$HOME/archivebot_ws/src/archivebot_description/worlds/archive_room.sdf" --render-engine ogre
~~~

Gazebo penceresi acilmali ve `archive_room` gorunmelidir. Simulasyon duraklatilmis ise **Play** tusuna basin.

### Terminal 2 — ROS 2 / Gazebo sensor ve hareket koprusu

~~~bash
source /opt/ros/lyrical/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$HOME/archivebot_ws/src/archivebot_description/config/bridge.yaml"
~~~

Bu kopru `/cmd_vel`, `/scan`, kamera, IMU, tampon, `/odom` ve `/clock` topic'lerini baglar.

### Terminal 3 — Gercek Gazebo koordinatini ROS 2'ye aktar

V11, diff-drive'in `/odom` topic'i yerine **`/archivebot/ground_truth/odom`** topic'ini kullanir. Bu ek kopru V11 icin gereklidir:

~~~bash
source /opt/ros/lyrical/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/archivebot/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
~~~

### Terminal 4 — Robot modelini uret ve Gazebo'ya ekle

~~~bash
source /opt/ros/lyrical/setup.bash

xacro "$HOME/archivebot_ws/src/archivebot_description/models/archivebot_v9.urdf.xacro" \
  -o /tmp/archivebot_v9.urdf

ros2 run ros_gz_sim create \
  -world archive_room \
  -file /tmp/archivebot_v9.urdf \
  -name archivebot \
  -x -4.5 -y -3.3 -z 0.225
~~~

`Entity creation successful` mesaji ve Gazebo'da robotun gorunmesi beklenir. **V11 navigasyonu icin V9 robot modeli kullanilir.** Robot zaten eklenmisse ikinci kez eklemeyin.

### Terminal 5 — Kamera renk algilama

~~~bash
source /opt/ros/lyrical/setup.bash
python3 "$HOME/archivebot_ws/src/archivebot_description/scripts/archivebot_vision.py"
~~~

### Terminal 6 — Onerilen navigasyonu baslat (V11)

Once Gazebo'nun Play durumunda oldugundan, robotun baslangic noktasinda bulundugundan ve koprulerin calistigindan emin olun.

~~~bash
source /opt/ros/lyrical/setup.bash
python3 "$HOME/archivebot_ws/src/archivebot_description/scripts/archivebot_navigation_v11.py"
~~~

Robot sirasiyla guney koridoruna, kirmizi ve yesil kontrol noktalarina ilerler; denetimlerden sonra baslangica doner, yonunu duzeltir ve `TAMAMLANDI` durumuna gecer. **Ayni anda sadece bir navigasyon surumu calistirin.**

## Sensor baglantilarini kontrol etme

Yeniden baslatilan ayri bir terminalde:

~~~bash
source /opt/ros/lyrical/setup.bash

ros2 topic info /scan
ros2 topic info /archivebot/camera/image
ros2 topic info /archivebot/ground_truth/odom

timeout 10s ros2 topic echo /scan --once --field header
timeout 10s ros2 topic echo /archivebot/ground_truth/odom --once --field pose.pose.position
~~~

Beklenen: topic'lerde publisher bulunmasi ve tek seferlik echo komutlarinin veri dondurmesi. Kamera denetimi icin `archivebot_vision.py` da calismalidir.

## Raporlar

Basarili kontrol noktalarinin CSV dosyasi:

~~~text
~/archivebot_ws/reports/archivebot_inspections.csv
~~~

Raporu terminalden okumak icin:

~~~bash
cat ~/archivebot_ws/reports/archivebot_inspections.csv
~~~

Raporlar ve derleme/gecici dosyalar `.gitignore` ile GitHub'a gonderilmez.

## Durdurma ve tekrar deneme

1. Navigasyon terminalinde `Ctrl+C` yapin; robot icin sifir hiz komutu yayinlanir.
2. Kamera ve kopru terminallerini `Ctrl+C` ile durdurun.
3. Gazebo'yu normal sekilde kapatin.
4. Yeniden deneyecekseniz Gazebo ve kopruleri baslatip robotu baslangic noktasina yeniden ekleyin.

Ayni Gazebo dunyasinda robot zaten mevcutsa once onu kaldirin:

~~~bash
ros2 run ros_gz_sim remove --ros-args \
  -p world:=archive_room -p entity_name:=archivebot
~~~

Ardindan Terminal 4'teki create komutunu tekrar kullanabilirsiniz.

## Bilinen sinirlamalar ve gelistirme plani

- VirtualBox ortaminda Gazebo'nun real-time factor (RTF) degeri dusuk olabiliyor; toplam gorev suresi bundan etkileniyor.
- V11'in test edilen tam gorev suresi, gelistirme VM'inde yaklasik **10 dakika 34 saniye** idi; her bilgisayarda ayni sure garanti edilmez.
- Gorevi **3–4 dakika** icinde bitirme hedefi ileriye donuk performans calismasidir.
- ROS 2 paket metadata dosyalari ve otomatik launch dosyalarinin eklenmesi planlanabilir.
- Guvenlik esiklerini rastgele dusurmeden once fren mesafesi ve sensor gecikmeleri test edilmelidir.

## Ekip calismasi

Projeyi guncellemek icin:

~~~bash
cd ~/archivebot_ws
git pull --ff-only origin main
~~~

Yeni ozellikleri ayri branch'te gelistirin, commit edin ve Pull Request acin. Dogrudan `main` uzerinde ayni dosyaya eszamanli degisiklik yapmaktan kacinmak faydalidir.
