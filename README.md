# Безопасно-интервальное планирование в среде с динамическими препятствиями и с дискретным временем (алгоритм SIPP и его модификации)

## Описание проекта

В этом проекте будет рассматриваться задача планирования траектории на сетчатых 4-connected графах с динамическими препятствиями. Целью проекта является реализация и/или сравнение алгоритмов:
- **AStar with timesteps**
- **SIPP**
- **Sub-optimal SIPP (WSIPP) with Re-expanded**
- **Sub-optimal SIPP (WSIPP) with Dublicate States**
- **Naive Anytime SIPP**

Все алгоритмы будут использовать манхэттеновскую эвристикy.

Алгоритм SIPP реализован без перераскрытий, так как доказано, что с такой эвристикой его решение будет оптимальным и без использования перераскрытий. Он будет сравниваться с AStar with timesteps.

Было решено, что наиболее интересной исследовательской задачей будет сравнение двух вариантов Sub-optimal SIPP.

Naive Anytime SIPP реализован с помощью последовательных запусков Sub-optimal SIPP с уменьшением параметра *w*. Было решено его реализовать, хотя он не является серьезным алгоритмом, так как он использует Sub-optimal SIPP.

## Установка и запуск

Скачайте репозиторий:
```Console
git clone https://github.com/MONICgh/SIPP-and-modifications.git
```

Убедитесь что у вас установлен `Jupyter Notebook` и `python3`. При необходимости можно установить следующим образом:
```Console
python3 -m pip install notebook
```

Запускаем `Jupyter Notebook`:
```Console
python3 -m notebook
```

И выбираем файл `SIPP.ipynb` нашего проекта.


## Литература

- Phillips, M. and Likhachev, M., 2011. SIPP: Safe interval path planning for dynamic environments. In 2011 IEEE International Conference on Robotics and Automation, ICRA 2011  (pp. 5628-5635). [**URL**](http://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf)

- Yakovlev, K., Andreychuk, A. and Stern, R., 2020, June. Revisiting bounded-suboptimal safe interval path planning. In Proceedings of the 20th International Conference on Automated Planning and Scheduling, ICAPS 2020 (pp. 300-304). [**URL**](https://ojs.aaai.org/index.php/ICAPS/article/download/6674/6528)

- Rybecky, T., Kulich, M., Andreychuk, A. and Yakovlev, K., 2021. Towards Narrowing the Search in Bounded-Suboptimal Safe Interval Path Planning. In Proceedings of the 12th International Symposium on Combinatorial Search (pp. 136-140). [**URL**](https://ojs.aaai.org/index.php/SOCS/article/download/18562/18351)

- Narayanan, V., Phillips, M. and Likhachev, M., 2012. Anytime safe interval path planning for dynamic environments. In 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS 2012 (pp. 4708-4715). [**URL**](http://www.ri.cmu.edu/pub_files/2012/10/asipp_iros12.pdf)

- Ren, Z., Rathinam, S., Likhachev, M. and Choset, H., 2022. Multi-Objective Safe-Interval Path Planning with Dynamic Obstacles. IEEE Robotics and Automation Letters, 7(3), pp.8154-8161. [**URL**](https://www.ri.cmu.edu/app/uploads/2022/07/ren22_mosipp_RAL_IROS22.pdf)
