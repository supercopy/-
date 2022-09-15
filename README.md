# -
新人入坑
sudhiuad
阿达
111
3032
#!/usr/bin/python
#coding=utf-8
'''
Created on 2012-2-22
Q: 给定一个列表，去掉其重复的元素，并输出
'''
def distFunc1():
  a=[1,2,4,2,4,5,6,5,7,8,9,0]
  b={}
  b=b.fromkeys(a)
  print b
  #print b.keys()
  a=list(b.keys())
  print a
def distFunc2():
  a=[1,2,4,2,4,5,7,10,5,5,7,8,9,0,3]
  a=list(set(a)) # set是非重复的，无序集合。可以用list来的排队对set进行排序，list()转换为列表，a.sort来排序
  print a
def distFunc3():
  #可以先把list重新排序，然后从list的最后开始扫描，代码如下：
  List=[1,2,4,2,4,5,7,10,5,5,7,8,9,0,3]
  if List:
    List.sort()
    #print List
    last = List[-1]
    #print last
    for i in range(len(List)-2, -1, -1):
      if last==List[i]: 
        del List[i]
      else: last=List[i]
if __name__ == '__main__':
  distFunc1()
  distFunc2()
  distFunc3()
