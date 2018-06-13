#include "language_struct.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

int trim(char *ptr,int len) 
{     
	int i,j;
	for(i=0;i<len;i++)     
	{  
		if(ptr[i]==' ')
		{           
			for(j=i;j<len;j++)             
			{ 
				ptr[j]=ptr[j+1];                    
			}
		}
	}
	return 0; 
}



int readWord(char *buf, enum wordType* type,int *len)
{
	printf("%s\n",buf);
	if(strncmp(buf,"MOVJ",4)==0)
	{
		*type=W_MOVJ;
		*len=4;
	}
	else if(strncmp(buf,"MOVL",4)==0)
	{
		*type=W_MOVL;
		*len=4;
	}
	else if(strncmp(buf,"MOVC",4)==0)
	{
		*type=W_MOVC;
		*len=4;
	}
	else if(strncmp(buf,"VJ=",3)==0)
	{
		*type=W_VJ;
		*len=3;
	}
	else if(strncmp(buf,"VL=",3)==0)
	{
		*type=W_VL;
		*len=3;
	}
	else if(strncmp(buf,"VR=",3)==0)
	{
		*type=W_VR;
		*len=3;
	}
	else if(strncmp(buf,"J1=",3)==0)
	{
		*type=W_J1;
		*len=3;
	}
	else if(strncmp(buf,"J2=",3)==0)
	{
		*type=W_J2;
		*len=3;
	}
	else if(strncmp(buf,"J3=",3)==0)
	{
		*type=W_J3;
		*len=3;
	}
	else if(strncmp(buf,"J4=",3)==0)
	{
		*type=W_J4;
		*len=3;
	}
	else if(strncmp(buf,"J5=",3)==0)
	{
		*type=W_J5;
		*len=3;
	}
	else if(strncmp(buf,"J6=",3)==0)
	{
		*type=W_J6;
		*len=3;
	}
	else if(strncmp(buf,"X=",2)==0)
	{
		*type=W_X;
		*len=2;
	}
	else if(strncmp(buf,"Y=",2)==0)
	{
		*type=W_Y;
		*len=2;
	}	
	else if(strncmp(buf,"Z=",2)==0)
	{
		*type=W_Z;
		*len=2;
	}
	else if(strncmp(buf,"A=",2)==0)
	{
		*type=W_A;
		*len=2;
	}
	else if(strncmp(buf,"B=",2)==0)
	{
		*type=W_B;
		*len=2;
	}
	else if(strncmp(buf,"C=",2)==0)
	{
		*type=W_C;
		*len=2;
	}
	else
	{
		return -1;
	}
	return 0;
}
int readValue(char* buf, double *value,int * len,int end)
{
	int i=0;
	int dot=0;
	int dotNum=0;
	double dotVal;
	*value=0;
	int j=0;
	for(i=0;i<end;i++)
	{
		//printf(" i=%d num= %c",i,buf[i]);
		if((buf[i]>='0'&&buf[i]<='9')||(buf[i]=='.'))
		{
			if(buf[i]=='.')
			{
				if(1==dot)
				{
					//printf("there are two dots in one number \n");
					return -1;
				}
					
				else
				{
					dot=1;
				}
			}
			else
			{
				if(1==dot)
				{
					dotVal=buf[i]-48;
					dotNum++;
					for(j=0;j<dotNum;j++)
					{
						dotVal=dotVal/10;
					}
					*value=*value+dotVal;	
				}
				else
				{
					*value=*value*10+buf[i]-48;
				}
			}
		}
		else
		{
			*len=i;
			return 0;
		}
			
	}

	return 0;
}

int readLine(char *buf, Line *line)
{
	int len;
	int start=0;
	int end=0;
	int end1=0;
	double value;
	int i=0;
	int ret=0;
	enum wordType type;
	len=strlen(buf);
	//printf("buf=%s,len=%d\n",buf,len);
	trim(buf,len);
	//printf("%s",buf);
	for(i=0;i<6;i++)
	{
		line->pos.jp.joint[i]=ABNORMAL_LARGE;
	}
	line->pos.p.x=ABNORMAL_LARGE;
	line->pos.p.y=ABNORMAL_LARGE;
	line->pos.p.z=ABNORMAL_LARGE;
	line->pos.p.a=ABNORMAL_LARGE;
	line->pos.p.b=ABNORMAL_LARGE;
	line->pos.p.c=ABNORMAL_LARGE;
	line->vel.angularVelocity=ABNORMAL_LARGE;
	line->vel.jointVelocity=ABNORMAL_LARGE;
	line->vel.linearVelocity=ABNORMAL_LARGE;
	
	
	while(readWord(buf+start,&type,&end)!=-1)
	{
		end1=0;		
		switch(type)
		{
			case W_MOVJ:
				line->cmdType=C_MOVJ;
				break;
			case W_MOVL:
				line->cmdType=C_MOVL;
				break;
			case W_MOVC:
				line->cmdType=C_MOVC;
				break;
			case W_J1:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.jp.joint[0]=value;
				}
				
				break;
			case W_J2:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.jp.joint[1]=value;
				}
				break;
			case W_J3:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.jp.joint[2]=value;
				}
				break;
			case W_J4:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.jp.joint[3]=value;
				}				
				break;
			case W_J5:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.jp.joint[4]=value;
				}					
				break;
			case W_J6:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.jp.joint[5]=value;
				}								
				break;
			case W_X:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.p.x=value;
				}
				break;
			case W_Y:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.p.y=value;
				}				
				break;
			case W_Z:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.p.z=value;
				}
				break;
			case W_A:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.p.a=value;
				}
				break;
			case W_B:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);				
				if(ret==0)
				{
					line->pos.p.b=value;
				}
				break;
			case W_C:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->pos.p.c=value;
				}
				break;
			case W_VJ:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->vel.jointVelocity=value;
				}		
				break;
			case W_VL:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->vel.linearVelocity=value;
				}
				break;
			case W_VR:
				ret=readValue(buf+start+end,&value,&end1,len-start-end);
				if(ret==0)
				{
					line->vel.angularVelocity=value;
				}			
				break;
			default:
				printf("unknown word type \n");
				break;
		}
		start+=end;
		start+=end1;
		//printf("start=%d, end=%d,end1=%d\n",start,end,end1);
	}


	
	return 0;
}

