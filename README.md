Future_Engineers_LIME
проект к WRO 2022 Comand LIME, г. Владивосток, категория Future Engineers

В разработке командного проекта LIME для конкурса WRO в номинации «Будущие инженеры» принимали участие Линейский Аким - программист, конструктор, Иван Терехов - инженер, электронщик, Антон Алексеевич - наш наставник и тренер.
 
Алгоритм программы
Робот был запрограммирован на Python версии 3.9.3. Интерпретатором был PyCharm.

Для соединения робота и компьютера возьмите кабель Ethernet, к которому подключен компьютер. Далее с помощью приложения StartRobot загружаем файл программы на микрокомпьютер Raspberry. Он сохраняет его на SD-карту, а затем выполняет код.

в основной программе нашего проекта мы используем библиотеки cv2, регуляторы, GPIORobot, numpy, time и другие.

Изначально получаем изображение с камеры 640х480. Дальше идем прямо, пока не найдем изображение в специальной маске HSV синего или оранжевого цвета не меньше определенного количества пикселей. От того, какой цвет мы увидим первым, мы поймем, в каком направлении нам предстоит двигаться в дальнейшем. После того, как мы определили направление движения, поворачиваем в соответствующем направлении. Если синяя линия первая, то направление движения против часовой стрелки, поворачиваем налево, если оранжевая линия первая, то от направления движения по часовой стрелке, поворачиваем налево. После в квалификации мы едем по регулятору до следующего поворота, а в финале одем до поворота с учетом распознавания и объезда знаков.

В финальной программе в отличие от квалификации есть ункции распознования знаков, алгоритм вычисления отъезда от них и логика составления карт времени и расстановки знаков
В квалификации все немного проще: отсутствуйют функции для знаков и нет соответствующей логики для вычисления отъезда. Мы идес в свём распоряжении датчики распознования чёрного бортика, синей и оранжевой линий на поворотах, функция отображения телеметрии данных. По большей части программа квалификации является основой для программы финала, ведь в финале необходимы все те баовые функции распознования и алгоритмы ориентации, счета линий что и в квалификации.

Наш алгоритм движения робота со знаками строится из нескольких этапов (стадий). Это: Старт, Самостоятельное движение, ручное управление. В спервом этапе - старт робот попадает сразу же после загрузки программы, там робот выравнивает сорвомотор и подает сигнал и загрузки. После нажатия на стартовую кнопку программа переходит во второй этап

Второй этап - Самостоятельное движение делится на 2 подчасти: регулятор движения и блок регулятора поворота и управления знаками.

В контроллере движения робот вырезает часть изображения, полученного камерой и по специальной маске HSV с левого или правого края, в зависимости от направления. Таким образом, у нас получается кусок черной линии — сторона, по которой мы будем идти. Мы определяем его координаты y и высоту контура. Складывая их вместе, мы получаем определенное числовое расстояние робота от края — борта. Во второй части - Знаки руления. С помощью HSV находим зеленый и красный знаки, а затем ближайший к нам — контур большего размера. Затем, зная направление и цвет ближайшего знака, решаем, в каком направлении нам нужно завернуть. Прибавляем это цисло к общей ошибке регулятора, в результате чего робот поворачивает в нужную сторону.

Остальные стадии алгоритма: Finish, Hand Control довольно просты.

Финиш - робот проходит определенное расстояние во времени, пройдя 12 поворотов - 3 круга.

Ручное управление - при нажатии стрелок на клавиатуре компьютера робот движется в соответствующем направлении или крутит серво - рули.

Чтобы загрузить в робота определенную программу, мы используем специальную программу под названием «Запуск робота». При его запуске открывается специальное окно с интерфейсом для передачи файлов, запуска программы и передачи видео с робота. В окне есть несколько кнопок: «Начать загрузку» — загружает определенную программу в робота. «Старт» — запускает программу, загруженную на робота. «Сырые» — запускает программу проверки «Видео» — запускает видео, полученное в реальном времени с камеры робота «Подключиться к роботу» — открывает меню доступных роботов для подключения

Мы используем сеть Wi-Fi и канал Ethernet для связи с роботом.

Системные требования для работы с программой Future Ingeneers:
Среда программирования - PyCharm.
Язык программирования - Python 3.9.
Доступ к репозиторию на Github и папке установленного проекта из репозитория
Bitvise SSH-клиент
наличие определенных файлов на Raspberry Pi
Операционная система компьютера, на котором вы будете работать, — Windows 10.
Установка необходимых компонентов:

Установка PyCharm:
а) Перейдите на официальный сайт PyCharm.
все изображение веб-сайта

б) Далее выберите операционную систему Windows. в) Нажмите кнопку загрузки. г) Далее будет загружен установочный файл образа

д) После установки откройте его и запустите файл е) Далее выберите нужные вам параметры.

Установка языка Python версии 3.9:
а) Перейдите на образ официального сайта python

б) Нажмите на изображение кнопки «Загрузить».

в) Прокрутите вниз, вы увидите список версий, которые можно скачать, выберите образ python 3.9

г) Вы увидите страницу этой версии, прокрутите вниз и выберите версию для образа 64-битной операционной системы

д) Далее идет установка языкового файла. f) Когда он будет установлен, запустите его, выберите нужные параметры и загрузите язык. g) Затем заходим в pychram, в левом верхнем углу будет изображение кнопки File

h) Затем нажмите на изображение кнопки настроек

i) Затем перейдите в этот раздел и выберите язык, на котором вы установили образ G.

Установка папки проекта из репозитория Github:
а) Вам нужно перейти по ссылке на образ репозитория Github

б) Затем нажмите на зеленую кнопку «Код», затем загрузите образ zip-архива

в) Должна начаться загрузка архива проекта image

г) Затем разархивируйте файл в обычный образ папки

Установка SSH-клиента Bitvise:
а) Заходим на официальный сайт программы: изображение

б) Нажмите на кнопку загрузки изображения

c) Нажмите на изображение кнопки Download Bitvise SSH Client.

d) Нажмите на изображение кнопки Download Bitvise SSH Client.

e) Затем начнется загрузка установочного файла.

f) По завершении загрузки откройте установочный файл и завершите установку программы.

5)Необходимо скачать файлы RobotAPI.py , autostart.py , INGENEERS.py на Raspberry Pi Подробная инструкция по установке по этой ссылке https://raspberrypi-ru.com/%D0%BF%D0%B5%D1%80%D0 %B5%D0%B4%D0%B0%D1%87%D0%B0-%D1%84%D0%B0%D0%B9%D0%BB%D0%BE%D0%B2-%D0%BC%D0 %B0%D0%BB%D0%B8%D0%BD%D0%B0-ssh/

Также необходимо загрузить в PyBoard файлы main.py, module.py. Для этого вам нужно подключить PyBoard к компьютеру с помощью кабеля Micro-USB. Затем переместите файлы main.py и module.py в микроконтроллер.

Запуск робота:

а) Вставьте аккумуляторы 3,7 В 18650 в батарейный отсек робота. Не путайте + и - батареи, чтобы избежать последствий.

б) Нажмите красную кнопку питания на роботе.

в) При включении микроконтроллера PyBoard будет звучать звук с повышением тона, дождитесь запуска микрокомпьютера Raspberry Pi, после завершения запуска он проиграет мелодию - двойную трель.

Загрузка проекта в робота:

а) Откройте PyCharm, нажмите кнопку «Файл» в левом верхнем углу. изображение

б) Затем нажмите на изображение кнопки Открыть

в) Далее, нажимая на значки папок в открывшемся окне, выберите папку, в которой находится разархивированный репозиторий GitHub проекта Future Engineers. изображение

г) По умолчанию папка должна быть по этому пути, можно скопировать и вставить в строку поиска C:\Users\user\Downloads\Future-Ingeneers image

e) Вы должны открыть изображение папки проекта

f) Соедините Raspberry Pi и компьютер кабелем Ethernet.

g) Теперь вернитесь в программу PyCharm, в меню файлов проекта выберите и откройте образ программы Start Robot

h) В правой части кнопок в верхней части экрана щелкните изображение кнопки «Выполнить».

i) Выберите кнопку «Выполнить», в открывшемся небольшом окне выберите образ программы «Запустить робота».

j) В этом окне откроется изображение

k) Чтобы загрузить программу на робота, нажмите кнопку «Загрузить Пуск» и выберите программу для запуска.

м) Чтобы начать запуск программы на роботе, нажмите кнопку Start image

m) Чтобы остановить программу на роботе, нажмите кнопку «Стоп».

n) Чтобы получить информацию о , нажмите на изображение кнопки Raw

o) Чтобы включить видеотрансляцию с робота, нажмите кнопку Видео изображение

р) Но перед использованием всех этих кнопок необходимо подключиться к роботу с помощью кнопки Подключиться к роботу. изображение

q) При нажатии на кнопку откроется список доступных вам роботов, выберите первый вариант изображения

т) Когда робот и компьютер соединятся друг с другом, появится надпись: image

Способы подключения к отдельным компонентам и обмена файлами: Raspberry Pi: для подключения к микрокомпьютеру Raspberry Pi необходимо подключить его к монитору через кабель HDMI для отображения изображения и подключить его к компьютеру через кабель Ethernet. Затем через командную строку с помощью команды Ifconfig получаем данные адреса Raspberry Pi. Найдите IP-адрес Ethernet. Введите адрес, пароль, порт и имя пользователя в программу Bitvise SSH Client. Таким образом, мы можем обмениваться файлами с Raspberry Pi.