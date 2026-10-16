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

**Paketleme notu:** Bu repository'de su an `package.xml` ve `CMakeLists.txt` yoktur. Bu nedenle asagidaki calistirma adimlari dosyalari dogrudan kullanir; henuz `colcon build` gerektiren bir ROS 2 paketi olarak dokumante edilmemistir.

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
