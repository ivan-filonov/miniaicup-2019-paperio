Описание правил можно прочитать по сслыке https://github.com/MailRuChamps/miniaicups/tree/master/paperio. Игровой мир
представляет собой квадрат со сторонами 930 на 930 условных единиц, разбитый на 31*31 элементарных ячеек, размером 30 на
30 каждая. Суть игры крайне проста - вы играете за умный квадрат, постоянно находящийся в движении, направление движения
которого вы можете менять. Также у вас есть небольшая территория, которую можно расширить, путем захвата новых клеток.
За захват новых клеток и за некоторые другие игровые действия будут начисляться очки. Надо набрать как можно больше
очков. Общение бота с игрой происходит через стандартные ввод-вывод.

Старт
В начале игры мы получаем json с описанием настроек игры, которые так ни разу и не менялись за все время и можно было
смело забивать их прямо в исходники. Далее, до конца игры в каждый момент времени, когда можем что-то сделать, получаем
описание состояния игроков и бонусов и отправляем в ответ команду в какую сторону двигаться.

Ходы
Так как данные каждый раз поступают в виде json со списками точек, принадлежащих игрокам и координатами игроков и
бонусов - их надо как-то импортировать и предобработать. В моем случае предобработка свелась к заполнению алгоритмом
Левита карт достижимости для каждого игрока с учетом его "хвоста", пересекать который запрещено. 
В более продвинутых стратегиях люди учитывали возможность, что игрок пройдет на свою территорию, что сделает "хвост"
частью его территории, и уже оттуда сможет двигаться без ограничений. Также у меня сознательно игнорировались
действующие бонусы на ускорение/замедление, но по ощущениям особых проблем это не доставляло.
Далее надо было решить куда двигаться. И чтобы не пересчитывать все на каждом ходу, я ввел понятие плана -
последовательности точек, которую надо пройти, если не нарушены внешние условия. Так что игровой цикл получался таким -
проверить актуальность текущего плана, если он не актуален - составить новый, получить очередную точку текущего плана.
Даже на слабом железе составление простых планов происходило достаточно быстро, чтобы вписаться в лимиты по времени,
даже пересчитывая планы каждый ход. К этому в итоге все и свелось.
Алгоритмы генерации планов - стратегии, создающие последовательности точек плана и условие прерывания выполнения плана.
Стратегии вызываются по очереди - если более приоритетная не сгенерировала план, то вызывается следующая. Если не
справилась ни одна - паникуем и выдаем случайный ход.
С некоторой вероятностью действующий план может быть прерван или быть сгенерирован не наиболее приоритетной стратегией.

Стратегия - прямоугольники
Берем текущую клетку, случайную клетку на поле, проводим прямоугольник с противоположными углами в этих двух клетках.
Обычно хватало времени обойти все поле, но иногда скорость оценки просаживалась настолько, что в итоге пришлось обходить
клетки в случайном порядке с ограничением по времени.
 План всегда строится так, чтобы заканчиваться на своей территории. Если в процессе выполнения плана конечная точка
 перестала быть своей - обнуляем план.  Для каждого варианта плана считаем закрашенные клетки и выгоду от них. Далее
 были варианты - максимизировать суммарную выгоду от плана или отношение выгода/время и учитывать ли во времени уже
 существующий хвост.

 Стратегия - паника
 Берется случайный набор возможных конечных клеток, строится путь до них. Оценивается безопасность пути, возможная
 выгода и конечная точка. Если начинаем паниковать на своей территории - ищем клетку снаружи рядом со своей границей,
 если начинаем снаружи - ищем клетку внутри. Выбираем самый безопасный маршрут. Если оценка опасности совпадает -
 выбираем с самой лучшей оценкой конечной точки.

 Стратегия - потенциальные поля
 Назначаем клеткам поля базовый вес в виде C / (оценка безопасности точки + K), так что наибольший вес - у клеток вокруг
 своей территории. Если в каких-то клетках враг может оказаться раньше, чем можем добежать к себе - обнуляем вес. Далее
 добавляем оценку веса содержимого соседних клеток, деленную на некоторую степень расстояния. Выбираем самую
 приоритетную клетку. В некоторых версиях домножаем веса клеток на случайное число из (0.95, 1.05) (различается в
 зависимости от версии).

 Стратегия - рандом
 Если ни одна осмысленная стратегия не нашла подходящего пути - остается только сделать что-нибудь случайное. То есть из
 3 возможных направлений выбираем случайное. Проверок на самоубийство об собственный хвост или стену при этом уже не
 производится - раз уж возникла необходимость в случайном ходе, значит ничего безопасного уже не осталось.