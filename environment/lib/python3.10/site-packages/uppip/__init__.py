#from start import startupgradepip
# Upgrade pip python packages from outdated

import sys, os, time
from sys import platform

def startupgradepip():
    print('''
    [+] Author : Xnuvers007 (from Indonesian)
    [+] Github : https://github.com/Xnuvers007/upgrade-pip
    [+] Instagram : Indradwi.25
    [+] Youtube : Xnuvers007
    [+] Website : https://mykingbee.blogspot.com
    [+] Portfolio : https://myportofolio.xnuvers007.repl.co
    [+] License : GNU/GPL
    [+] Donate : https://saweria.co/xnuvers007
    [+] Donate 2 : https://trakteer.id/Xnuvers007
        ''')
    localtime = time.asctime(time.localtime(time.time()))
    print("\n[+] Current time :", localtime,"\n")
    if platform=="win32":
        print("checking for update / outdated pip packages")
        print("it will take 5-10 minutes for checking\n")
        time.sleep(2)
        os.system("pip list --outdated")
        print("what do u want to uptade ?")
        print("1. all")
        print("2. specific")
        print("3. exit")
        choice = input("enter your choice: ")
        if choice == "1":
            os.system("pip freeze | %{$_.split('==')[0]} | %{pip install --upgrade $_}")

        elif choice == "2":
            print("enter the package name")
            package = input("enter the package name: ")
            os.system("pip install --upgrade "+package)
        elif choice == "3":
            print("exiting")
            sys.exit(1)

    elif platform=="linux" or platform=="linux2":
        termux = str(input("do you use termux ? [y/n]: "))
        if termux == "y" or termux == "Y":
            print("checking for update / outdated pip packages")
            print("it will take 5-10 minutes for checking\n")
            time.sleep(2)
            os.system("pip list --outdated")
            print("what do u want to uptade ?")
            print("1. all")
            print("2. specific")
            print("3. exit")
            choice = input("enter your choice: ")
            if choice == "1":
                os.system("pip list --outdated --format=freeze | grep -v '^\-e' | cut -d = -f 1 | xargs -n1 pip install -U")
                tanya = str(input("do you want to exit ? [y/n]: "))
                if tanya == "y" or tanya == "Y":
                    for i in range(4):
                        i += 1
                        print("exiting in "+str(i)+" seconds")
                        time.sleep(1)
                    sys.exit(1)
                elif tanya == "n" or tanya == "N":
                    startupgradepip()
                else:
                    print("wrong input")
                    sys.exit(1)
            elif choice == "2":
                print("enter the package name")
                package = input("enter the package name: ")
                os.system("pip install --upgrade "+package)
                tanya = str(input("do you want to exit ? [y/n]: "))
                if tanya == "y" or tanya == "Y":
                    for i in range(4):
                        i += 1
                        print("exiting in "+str(i)+" seconds")
                        time.sleep(1)
                    sys.exit(1)
                elif tanya == "n" or tanya == "N":
                    startupgradepip()
                else:
                    print("wrong input")
                    sys.exit(1)
            elif choice == "3":
                print("exiting")
                sys.exit(1)
            else:
                print("wrong input")
                sys.exit(1)
        elif termux == "n" or termux == "N":
            # if distro kali, then using apt get install grep, if distro centos7, then using yum install grep
            import distro
            if distro.id()=="debian" or distro.id()=="ubuntu" or distro.id()=="linuxmint":
                os.system("sudo apt-get install grep")
            elif distro.id()=="centos" or distro.id()=="rhel":
                os.system("sudo yum install grep")
            elif distro.id()=="arch":
                os.system("sudo pacman -S grep")
            elif distro.id()=="fedora":
                os.system("sudo dns install grep")
            elif distro.id()=="opensuse":
                os.system("sudo zypper install grep")
            else:
                pass

            print("checking for update / outdated pip packages")
            print("it will take 5-10 minutes for checking\n")
            time.sleep(2)
            os.system("sudo pip3 list --outdated")
            print("what do u want to update ?")
            print("1. all")
            print("2. specific")
            print("3. exit")
            choice = input("enter your choice: ")
            if choice == "1":
                os.system("sudo pip3 list --outdated --format=freeze | grep -v '^\-e' | cut -d = -f 1 | xargs -n1 sudo pip3 install -U")
                tanya = str(input("do you want to exit ? (y/n)"))
                if tanya=="y" or tanya=="Y":
                    for i in range(4):
                        i += 0
                        print("exiting in "+str(4-i)+" seconds")
                        time.sleep(1)
                    sys.exit(1)
                elif tanya=="n" or tanya=="N":
                    startupgradepip()
                else:
                    print("wrong input")
                    sys.exit(1)
                    
            elif choice == "2":
                print("enter the package name")
                package = input("enter the package name: ")
                os.system("sudo pip3 install --upgrade "+package)
                # if done, then user go to install again
                tanya = str(input("do you want to exit ? (y/n)"))
                if tanya=="y" or tanya=="Y":
                    for i in range(4):
                        i += 0
                        print("exiting in "+str(4-i)+" seconds")
                        time.sleep(1)
                    sys.exit(1)
                elif tanya=="n" or tanya=="N":
                    startupgradepip()
                else:
                    print("wrong input")
                    sys.exit(1)
            elif choice == "3":

                print("exiting")
                sys.exit()

        else:
            print("platform not supported")
            sys.exit(1)

# def main():
#     startupgradepip()

# if __name__ == "__main__":
#     main()