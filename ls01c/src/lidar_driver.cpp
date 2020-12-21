#include "lidar_driver.h"


#define DEBUG  0

#if DEBUG
#define ALOGI(x...)     printf( x)
#else
#define ALOGI(x...)    
#endif

static pthread_mutex_t g_tMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_tConVar  = PTHREAD_COND_INITIALIZER;
static pthread_t g_pthread;

static int g_fd;
static struct basedata *g_pcurr = NULL;

static double g_angle[PACKLEN];
static double g_distance[PACKLEN];
static double g_speed;
static int g_packlen;
static int g_run = 1;

/*******************************************************************
  * 函数名称:  VoltageControl
  * 功能描述:  控制雷达开始扫瞄
  * 函数参数:  0: 关闭, 非0:打开
  * 返回值     :   大于0成功，小于0失败
  * 修改日期:       版本号        修改人      修改内容
  *-------------------------------------------------------------------
  *  2017/04/10               v001.01                           增加电压控制命令
  ********************************************************************/
static int VoltageControl(int cmd)
{
	int ret;
	
	unsigned char open_cmd[] = 
	{0xa5, 0x3a, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd};
	unsigned char close_cmd[] = 
	{0xa5, 0x39, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd};
	
	usleep(1000000);
	if (cmd == 0)
	{
		ret =  write(g_fd, close_cmd, sizeof(close_cmd));
	}
	else
	{
		ret =  write(g_fd, open_cmd, sizeof(open_cmd));
	}

	return ret;
}

/************************************************************
  * 函数名称:  LidarStartScan
  * 功能描述:  控制雷达开始扫瞄
  * 函数参数:  无
  * 返回值     :   大于0成功，小于0失败
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01              
  ************************************************************/
static int LidarStartScan(void)
{
	int wRet;

	unsigned char cmd1[] =
	{ 0xa5, 0x2C, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };
	unsigned char cmd2[] =
	{ 0xa5, 0x20, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };


	wRet = VoltageControl(1);
	if (wRet < 0)
		return wRet;
	
	usleep(1000000);
	wRet = write(g_fd, cmd1, sizeof(cmd1));
	if (wRet < 0)
		return wRet;

	usleep(200000);
	wRet = write(g_fd, cmd2, sizeof(cmd2));
	if (wRet < 0)
		return wRet;

	return wRet;
}


 /************************************************************
  * 函数名称:  LidarStopScan
  * 功能描述:  控制雷达停止扫瞄
  * 函数参数:  无
  * 返回值     :   大于0成功，小于0失败
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01              
  ************************************************************/
static int LidarStopScan(void)
{
	unsigned char stop_scan[] =
	{ 0xa5, 0x21, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };

	usleep(200000);
	return write(g_fd, stop_scan, sizeof(stop_scan));
}

 /************************************************************
  * 函数名称:  LidarStopMotor
  * 功能描述:  控制雷达停止转动
  * 函数参数:  无
  * 返回值     :   大于0成功，小于0失败
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
static int LidarStopMotor(void)
{
	int ret;
	
	unsigned char stop_motor[] =
	{ 0xa5, 0x25, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };

	usleep(200000);
	ret = write(g_fd, stop_motor, sizeof(stop_motor));
	if (ret < 0)
		return ret;

	return VoltageControl(0);
}

 /************************************************************
  * 函数名称:  LidarReset
  * 功能描述:  控制雷达复位
  * 函数参数:  无
  * 返回值     :   大于0成功，小于0失败
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01              
  ************************************************************/
static int LidarReset(void)
{
	int ret;
	
	unsigned char buf[] =
	{ 0xa5, 0x40, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };

	ret = VoltageControl(1);
	if (ret < 0)
		return ret;
	
	usleep(200000);
	return write(g_fd, buf, sizeof(buf));
}

/************************************************************
  * 函数名称:  CreatList
  * 功能描述:  创建缓存
  * 输入参数:  无
  * 输出参数:  无
  * 返回值     :  不为NULL成功，NULL为失败 
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
static struct basedata *CreatList(void)
{
	struct basedata *head;

	head = (struct basedata *) malloc(sizeof(struct basedata));
	if (NULL == head)
		return NULL;
	
	head->flag = 0;
	head->start = 0;
	head->end = 0;
	head->curr = 0;
	head->next = NULL;

	return head;
}

/************************************************************
  * 函数名称:  InitList
  * 功能描述:  创建链表
  * 输入参数:  无
  * 输出参数:  无
  * 返回值     :  不为NULL成功，NULL为失败 
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
static struct basedata *InitList(void)
{
	struct basedata *head, *p;

	head = CreatList();
	if (NULL == head)
		return NULL;
	
	p = CreatList();
	if (NULL == p)
	{
		free(head);
		return NULL;
	}
	
	head->next = p;
	p->next = head;

	return head;
}

 /******************************************************************
  * 函数名称:  LidarParse
  * 功能描述:   解析雷达一圈数据
  * 输出参数:  1:雷达一圈数据2:角度3:距离4:一圈大小
  * 返回值     :  无 
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------------
  *  2017/04/10               v001.01               
  ******************************************************************/
static void LidarParse(unsigned char *data, double *angle, double *dist, int len)
{
	int i, j;
	unsigned char *tmp;
	lslidar_response_measurement_node_t *curr;
	
	ALOGI("len = %d, data[3610] = %02x\n", len, data[3610]);
	if (data[0] == 0xA5 && data[6] == 0x81 && data[len - 1] == 0xdd)
	{
		tmp = data + 7;
		g_speed = data[1] / 15.0;
		curr = (lslidar_response_measurement_node_t *) tmp;
		for (i = 7, j = 0; i < len - 4 && j < PACKLEN; curr++, i += 5, j++)
		{
			//ALOGI("%d  ", curr->sync_quality);
			ALOGI("%d  ", curr->angle_q6_checkbit);
			angle[j] = curr->angle_q6_checkbit / 10.0;
			
			ALOGI("%d  \n", curr->distance_q2);
			dist[j] = curr->distance_q2 / 1.0;
		
		}
		
		if (j >= PACKLEN)
		{
			pthread_mutex_lock(&g_tMutex);
			pthread_cond_signal(&g_tConVar);
			g_packlen = len;
			j = 0;
			pthread_mutex_unlock(&g_tMutex);
		}
		
	}

}

/*************************************************************
  * 函数名称:  LidarData
  * 功能描述:   获得雷达一圈数据
  * 函数参数:  1:获得雷达原始数据2:数组大小
  * 返回值     :  无 
  * 修改日期:       版本号        修改人      修改内容
  *------------------------------------------------------------
  *  2017/04/10               v001.01               
  *************************************************************/
static void LidarData(unsigned char *buf, int nRet)
{
	int i;
	int total = 0;
	unsigned char tempbuffer[2048];

	if (nRet > 0)
	{
		if (!g_pcurr->start && !g_pcurr->flag)
		{
			for (i = 0; i < nRet - 6; i++)
			{
				if (buf[i] == 0xa5 && buf[i + 6] == 0x81)
				{
					break;
				}
			}

			ALOGI("i0 = %d\n", i);
			
			if (i >= nRet - 6)
			{
				memcpy(g_pcurr->data, buf + nRet - 6, 6);
				g_pcurr->flag = 1;
				g_pcurr->curr = 6;
			}
			else
			{
				memcpy(g_pcurr->data, buf + i, nRet - i);
				g_pcurr->start = 1;
				g_pcurr->flag = 1;
				g_pcurr->curr = nRet - i;
			}
		}
		else if (!g_pcurr->start && g_pcurr->flag)
		{
			memset(tempbuffer, 0, sizeof(tempbuffer));
			memcpy(tempbuffer, g_pcurr->data, g_pcurr->curr);
			memcpy(tempbuffer + g_pcurr->curr, buf, nRet);
			
			total = g_pcurr->curr + nRet;
			ALOGI("total=%d,nRet=%d\n", total, nRet);
			g_pcurr->start = 0;
			g_pcurr->end = 0;
			g_pcurr->flag = 0;
			g_pcurr->curr = 0;
			memset(g_pcurr->data, 0, PACKSIZE);
			
			for (i = 0; i < total - 6; i++)
			{
				if (tempbuffer[i] == 0xa5 && tempbuffer[i + 6] == 0x81)
				{
					break;
				}
			}
			
			ALOGI("i1=%d\n", i);
			
			if (i >= total - 6)
			{
				memcpy(g_pcurr->data, tempbuffer + total - 6, 6);
				g_pcurr->flag = 1;
				g_pcurr->curr = 6;
			}
			else
			{
				if (total - i < PACKSIZE)
				{
					memcpy(g_pcurr->data, tempbuffer + i, total - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->curr = total - i;
				}
				else if (total - i == PACKSIZE)
				{
					memcpy(g_pcurr->data, tempbuffer + i, total - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->end = 1;
					g_pcurr->curr += total - i;
				}
				else
				{
					if (tempbuffer[i + PACKSIZE] == 0xa5)
					{
						memcpy(g_pcurr->data, tempbuffer + i, PACKSIZE);
						g_pcurr->start = 1;
						g_pcurr->flag = 1;
						g_pcurr->end = 1;
						g_pcurr->curr = PACKSIZE;
						g_pcurr = g_pcurr->next;
						g_pcurr->start = 0;
						g_pcurr->flag = 0;
						g_pcurr->end = 0;
						g_pcurr->curr = 0;
						memset(g_pcurr->data, 0, PACKSIZE);
						memcpy(g_pcurr->data, tempbuffer + i + PACKSIZE, total - i - PACKSIZE);
						g_pcurr->start = 0;
						g_pcurr->flag = 1;
						g_pcurr->end = 0;
						g_pcurr->curr = total - i - PACKSIZE;
						g_pcurr = g_pcurr->next;
					}
					else
					{
						memcpy(g_pcurr->data, tempbuffer + i + 1, total - i - 1);
						g_pcurr->start = 0;
						g_pcurr->flag = 1;
						g_pcurr->curr = total - i - 1;
					}
				}
			}
		}
		else if (g_pcurr->start && !g_pcurr->end)
		{
			for (i = 0; i < nRet - 6; i++)
			{
				if (buf[i] == 0xa5 && buf[i + 6] == 0x81)
				{
					break;
				}
			}

			ALOGI("i2=%d,nRet=%d\n",i,nRet);
			
			if (i >= nRet - 6)
			{
				if (g_pcurr->curr + i < PACKSIZE)
				{
					if (g_pcurr->curr + nRet < PACKSIZE)
					{
						memcpy(g_pcurr->data + g_pcurr->curr, buf, nRet);
						g_pcurr->curr += nRet;
					}
					else if (g_pcurr->curr + nRet == PACKSIZE)
					{
						memcpy(g_pcurr->data + g_pcurr->curr, buf, nRet);
						g_pcurr->curr += nRet;
						g_pcurr->end = 1;
					}
					else
					{
						total = PACKSIZE - g_pcurr->curr;
						if (buf[total] == 0xa5)
						{
							memcpy(g_pcurr->data + g_pcurr->curr, buf, total);
							g_pcurr->end = 1;
							g_pcurr->curr += total;
							g_pcurr = g_pcurr->next;
							g_pcurr->start = 0;
							g_pcurr->end = 0;
							g_pcurr->flag = 0;
							memset(g_pcurr->data, 0, PACKSIZE);
							memcpy(g_pcurr->data, buf + total, nRet - total);
							g_pcurr->start = 0;
							g_pcurr->curr = nRet - total;
							g_pcurr->end = 0;
							g_pcurr->flag = 1;
							g_pcurr = g_pcurr->next;
						}
						else
						{
							g_pcurr->start = 0;
							g_pcurr->end = 0;
							g_pcurr->flag = 0;
							memset(g_pcurr->data, 0, PACKSIZE);
							memcpy(g_pcurr->data, buf + nRet - 6, 6);
							g_pcurr->start = 0;
							g_pcurr->flag = 1;
							g_pcurr->curr = 6;
						}
					}
				}		
				else if (g_pcurr->curr + i == PACKSIZE)
				{
					if (buf[i] == 0xa5)
					{
						memcpy(g_pcurr->data + g_pcurr->curr, buf, i);
						g_pcurr->curr += i;
						g_pcurr->end = 1;
						g_pcurr = g_pcurr->next;
						g_pcurr->start = 0;
						g_pcurr->end = 0;
						g_pcurr->flag = 0;
						memset(g_pcurr->data, 0, PACKSIZE);
						memcpy(g_pcurr->data, buf + i, nRet - i);
						g_pcurr->start = 0; /* no start*/
						g_pcurr->flag = 1;
						g_pcurr->curr = nRet - i;
						g_pcurr = g_pcurr->next;
					}
					else
					{

						g_pcurr->start = 0;
						g_pcurr->end = 0;
						g_pcurr->flag = 0;
						memset(g_pcurr->data, 0, PACKSIZE);
						memcpy(g_pcurr->data, buf + nRet - 6, 6);
						g_pcurr->start = 0;
						g_pcurr->flag = 1;
						g_pcurr->curr = 6;
					}
				}
				else
				{             //(g_pcurr->curr+i > PACKSIZE)
					g_pcurr->start = 0;
					g_pcurr->end = 0;
					g_pcurr->flag = 0;
					memset(g_pcurr->data, 0, PACKSIZE);
					memcpy(g_pcurr->data, buf + nRet - 6, 6);
					g_pcurr->start = 0;
					g_pcurr->flag = 1;
					g_pcurr->curr = 6;

				}
			}
			else
			{
				if (g_pcurr->curr + i != PACKSIZE)
				{
					g_pcurr->start = 0;
					g_pcurr->end = 0;
					g_pcurr->flag = 0;
					memset(g_pcurr->data, 0, PACKSIZE);
					memcpy(g_pcurr->data, buf + i, nRet - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->curr = nRet - i;
				}
				else
				{
					memcpy(g_pcurr->data + g_pcurr->curr, buf, i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->end = 1;
					g_pcurr->curr += i;
					g_pcurr = g_pcurr->next;
					memcpy(g_pcurr->data, buf + i, nRet - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->end = 0;
					g_pcurr->curr = nRet - i;
					g_pcurr = g_pcurr->next;
				}
			}
		}
		
		if (g_pcurr->start && g_pcurr->end)
		{
			LidarParse(g_pcurr->data, g_angle, g_distance, g_pcurr->curr);
			g_pcurr->start = 0;
			g_pcurr->end = 0;
			g_pcurr->flag = 0;
			memset(g_pcurr->data, 0, PACKSIZE);
			g_pcurr = g_pcurr->next;
		}
	}
}

 /************************************************************
  * 函数名称:  CreatePthread
  * 功能描述:  创建线程
  * 函数参数:  1:线程数据
  * 返  回   值 :  无 
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
static void *CreatePthread(void *data)
{

	unsigned char buf[1024];
	fd_set read_fds;
	struct timeval tm;
	int nRet;

	while (g_run)
	{
		FD_ZERO(&read_fds);
		FD_SET(g_fd, &read_fds);
		
		tm.tv_sec = 0;
		tm.tv_usec = 300000;
		
		nRet = select(g_fd + 1, &read_fds, NULL, NULL, &tm);
		if (nRet < 0)
		{
			pthread_mutex_lock(&g_tMutex);
			pthread_cond_signal(&g_tConVar);
			ALOGI("select error!\n");
			g_packlen = -2;
			pthread_mutex_unlock(&g_tMutex);
		}
		else if (nRet == 0)
		{
			pthread_mutex_lock(&g_tMutex);
			pthread_cond_signal(&g_tConVar);
			ALOGI("timeout\n");
			g_packlen = 0;
			pthread_mutex_unlock(&g_tMutex);
		}
		else
		{
			if (FD_ISSET(g_fd, &read_fds))
			{
				bzero(buf, 1024);
				nRet = read(g_fd, buf, 1024);
				if (nRet > 0)
				{
					LidarData(buf, nRet);
					usleep(30000);
				}
				else 
				{
					pthread_mutex_lock(&g_tMutex);
					pthread_cond_signal(&g_tConVar);
					ALOGI("read error\n");
					g_packlen = -1;
					pthread_mutex_unlock(&g_tMutex);
				}
			}
		}

	}
	
	return NULL;
	
}

/************************************************************
  * 函数名称:  OpenLidarSerial
  * 功能描述:   打开串口
  * 函数参数:  1:串口节点2:串口波特率
  * 返  回   值 :  大于0成功，小于0失败 
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
int lidar_driver::OpenLidarSerial(const char* port, unsigned int baudrate)
{
	struct termios m_stNew;
	struct termios m_stOld;

	const char* addr = port;
	const char* addr2 = port;

	g_fd = open(addr, O_RDWR | O_NOCTTY | O_NDELAY);
	if (-1 == g_fd)
	{
		usleep(30000);
		g_fd = open(addr2, O_RDWR | O_NOCTTY | O_NDELAY);
		if (g_fd < 0)
			return -1;
	}
	
	ALOGI("start init serial\n");
	
	if ((fcntl(g_fd, F_SETFL, 0)) < 0)
	{
		perror("Fcntl F_SETFL Error!\n");
		return -1;
	}
	if (tcgetattr(g_fd, &m_stOld) != 0)
	{
		perror("tcgetattr error!\n");
		return -1;
	}

	m_stNew = m_stOld;
	cfmakeraw(&m_stNew);		    

	//set speed
	cfsetispeed(&m_stNew, baudrate);		 
	cfsetospeed(&m_stNew, baudrate);

	//set databits
	m_stNew.c_cflag |= (CLOCAL | CREAD);
	m_stNew.c_cflag &= ~CSIZE;
	m_stNew.c_cflag |= CS8;

	//set parity
	m_stNew.c_cflag &= ~PARENB;
	m_stNew.c_iflag &= ~INPCK;

	//set stopbits
	m_stNew.c_cflag &= ~CSTOPB;
	m_stNew.c_cc[VTIME] = 0;	
	m_stNew.c_cc[VMIN] = 1;	
	
	tcflush(g_fd, TCIFLUSH);	
	if (tcsetattr(g_fd, TCSANOW, &m_stNew) != 0)
	{
		perror("tcsetattr Error!\n");
		return -1;
	}
	
	g_pcurr = InitList();
	if (NULL == g_pcurr)
		return -1;
	
	g_run = 1;

	pthread_create(&g_pthread, NULL, CreatePthread, NULL);

	ALOGI("finish init seria!\n");
	
	return g_fd;
}

/************************************************************
  * 函数名称:  SendLidarCommand
  * 功能描述:   向雷达发送控制命令
  * 函数参数:  1:雷达控制命令
  * 返   回  值 :  大于0成功，小于0失败 
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
int lidar_driver::SendLidarCommand(Command cmd)
{
	int ret = -1;
	
	switch(cmd)
	{
		case START_SCAN:
			ret = LidarStartScan();
			break;
		case STOP_DATA:
			ret = LidarStopScan();
			break;
		case STOP_MOTOR:
			ret = LidarStopMotor();
			break;
		case STOP_MOTOR_AND_DATA:
			ret = LidarStopScan();
			if (ret < 0)
				break;
			ret = LidarStopMotor();
			break;
		case RESET:
			ret = LidarReset();
			break;
		default:
			break;
	}

	return ret;
}

/************************************************************
  * 函数名称:  GetLidarScanData
  * 功能描述:    获得雷达一圈数据
  * 函数参数:  1:角度2:距离3:数组大小4:雷达转速
  * 返   回  值 :  大于0成功，等于0超时 ,小于0出错
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01              
  ************************************************************/
int lidar_driver::GetLidarScanData(double *angle, double *distance, int len, double *speed)
{
	int min = 0;
	int i;
	
	pthread_mutex_lock(&g_tMutex);
	pthread_cond_wait(&g_tConVar, &g_tMutex);

	if (g_packlen > 0)
	{
		min = len > PACKLEN ? PACKLEN : len;
		for (i = 0; i < min; i++)
		{
			angle[i] = g_angle[i];
			distance[i] = g_distance[i];
		}
		
		*speed = g_speed;
	}
	else 
	{
		min = g_packlen;
	}
	
	g_packlen = 0;
	
	pthread_mutex_unlock(&g_tMutex);

	return min;
}

/************************************************************
  * 函数名称:  CloseLidarSerial
  * 功能描述:  关闭串口
  * 函数参数:  无
  * 返 回    值 :  无
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
void lidar_driver::CloseLidarSerial(void)
{
	struct basedata *tmp, *head;
	
	g_run  = 0;

	pthread_join(g_pthread, NULL);
	
	head = g_pcurr;
	g_pcurr = g_pcurr->next;
	
	while (g_pcurr != head)
	{
		tmp = g_pcurr;
		g_pcurr = g_pcurr->next;
		free(tmp);
		tmp = NULL;
	}
	
	free(head);
	g_pcurr = head = NULL;
	
	close(g_fd);
	
}

