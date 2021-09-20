import numpy as np
import matplotlib.pyplot as plt
import datetime

MONTH = 4
DATE_start = datetime.datetime.now() - datetime.timedelta(days = 3)
DATE_end = datetime.datetime.now()
# DATE_start = datetime.datetime(2021, 7, 27,17, 43)
# DATE_end = datetime.datetime(2021, 7, 27,17, 54)


data_cap = []
data_voltage = []
data_current = []
data = []
time = []
data_t1 = []
data_t2 = []
filepath = 'log.txt'
with open(filepath) as fp:
   line = fp.readline()

   while line:
       
        try:
            line = fp.readline()


            try:
                get_date = line.strip().split()[0]
                get_time = line.strip().split()[1]
                yy = int(get_date.split('-')[0])
                MM = int(get_date.split('-')[1])
                dd = int(get_date.split('-')[2])
                hh = int(get_time.split(':')[0])
                mm = int(get_time.split(':')[1])
                ss = int(get_time.split(':')[2].split('.')[0])
                # mss = int(get_time.split(':')[2].split('.')[1])
                # print('hh',hh)


                # if dd >= DATE_start and dd <= DATE_end:
                # date
                if datetime.datetime(yy,MM,dd,hh,mm,ss) >= DATE_start and datetime.datetime(yy,MM,dd,hh,mm,ss) <= DATE_end:

                    data_ok = True

                    try:
                        t1 = float(line.strip().split()[2])
                    except:
                        data_ok = False

                    try:
                        t2 = float(line.strip().split()[3])
                    except:
                        data_ok = False

                    try:
                        v = float(line.strip().split()[4])
                    except:
                        data_ok = False

                    try:
                        c = float(line.strip().split()[5])
                    except:
                        data_ok = False

                    try:
                        cap = float(line.strip().split()[6])
                    except:
                        data_ok = False



                    if data_ok == True:
                        time.append(datetime.datetime(yy,MM,dd,hh,mm,ss))
                        data_voltage.append(v)
                        data_current.append(c)
                        data_t1.append(t1)
                        data_t2.append(t2)
                        data_cap.append(cap)


            except Exception as e:
                print(e)
                pass

            # print(type(line.strip().split()[7]))
            # print(line.strip().split()[7])
        except Exception as e:
            print(e)


# time_current = np.arange(0, len(data_current))


fig, ax = plt.subplots(3, 1, figsize=(9,8))

# plt.subplot(2, 1, 1)
ax[0].plot(time, data_cap, 'g')
ax[0].set_title('battery level',fontsize=10)
ax[0].grid(True)
ax[0].tick_params(axis='x', labelrotation=20)

ax[1].plot(time, data_voltage, 'b')
ax[1].set_title('Voltage',fontsize=10)
ax[1].grid(True)
ax[1].tick_params(axis='x', labelrotation=20)

ax[2].plot(time, data_current, 'r')
ax[2].set_title('Current',fontsize=10)
ax[2].grid(True)
ax[2].tick_params(axis='x', labelrotation=20)

# ax[3].plot(time, data_t1, 'r')
# ax[3].plot(time, data_t2, 'g')
# ax[3].set_title('Temperature',fontsize=10)
# ax[3].grid(True)
# ax[3].tick_params(axis='x', labelrotation=20)
# ax[3].legend(['t1','t2'])
# fig.autofmt_xdate()
plt.show()
# 
