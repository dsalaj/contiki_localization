DEFINES=PROJECT_CONF_H=\"project-conf.h\"

# Define the target platform
ifndef TARGET
TARGET=sky
endif

# Use the RIME networking stack
CONTIKI_WITH_RIME=1

# Give a name to your project 
CONTIKI_PROJECT = exercise_1

# Compile project typing "make"
all: $(CONTIKI_PROJECT)

# Upload project typing "make upload"
upload: $(CONTIKI_PROJECT).upload

# Reset nodes
reset: sky-reset

# Read the serial port: write "make serial PORT=6" to connect to the /dev/com6 in Windows or to the /dev/ttyUSB6 in Linux
serial:
ifdef OS
	$(CONTIKI)/tools/sky/serialdump-windows -T -b115200 /dev/com$(PORT)	
else
	$(CONTIKI)/tools/sky/serialdump-linux -T -b115200 /dev/ttyUSB$(PORT)
endif

# Define additional source files to be compiled (if any)
#CONTIKI_TARGET_SOURCEFILES += settings_cc2420_rssi.c

#Normal Contiki
CONTIKI = ../..
include $(CONTIKI)/Makefile.include

# Including Math library
CUSTOM_RULE_LINK=1
%.$(TARGET): %.co $(PROJECT_OBJECTFILES) $(PROJECT_LIBRARIES) contiki-$(TARGET).a
	$(LD) $(LDFLAGS) $(TARGET_STARTFILES) ${filter-out %.a,$^} ${filter %.a,$^} $(TARGET_LIBFILES) -o $@ -lm
	
	
