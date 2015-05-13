obj-y += io.o dma.o
ifndef CONFIG_ARM_ARCH_TIMER
obj-y += timer.o
endif
obj-y += clock.o clock-voter.o clock-dummy.o
obj-y += modem_notifier.o subsystem_map.o
obj-$(CONFIG_CPU_FREQ_MSM) += cpufreq.o
obj-$(CONFIG_DEBUG_FS) += clock-debug.o

obj-$(CONFIG_MSM_PROC_COMM) += proc_comm.o
obj-$(CONFIG_MSM_PROC_COMM) += clock-pcom.o
obj-$(CONFIG_MSM_PROC_COMM) += vreg.o mpp.o
ifdef CONFIG_MSM_PROC_COMM
	obj-$(CONFIG_REGULATOR) += footswitch-pcom.o
	obj-$(CONFIG_DEBUG_FS) += pmic_debugfs.o
endif

obj-y += acpuclock.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += pmu.o

ifndef CONFIG_MSM_SMP
obj-$(CONFIG_ARCH_MSM_SCORPION) += msm_fault_handlers.o
endif

obj-$(CONFIG_MSM_VIC) += irq-vic.o

obj-$(CONFIG_CPU_V7) += idle-v7.o

quiet_cmd_mkrpcsym = MKCAP   $@
      cmd_mkrpcsym = $(PERL) $(srctree)/$(src)/mkrpcsym.pl $< $@

targets += smd_rpc_sym.c
$(obj)/smd_rpc_sym.c: $(src)/smd_rpc_sym $(src)/mkrpcsym.pl
	$(call if_changed,mkrpcsym)

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MSM_SMD_LOGGING) += smem_log.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o remote_spinlock.o smd_private.o
obj-y += socinfo.o
obj-$(CONFIG_MTD_MSM_NAND) += nand_partitions.o
obj-$(CONFIG_MSM_SMD_TTY) += smd_tty.o
obj-$(CONFIG_MSM_SMD_QMI) += smd_qmi.o
obj-$(CONFIG_MSM_SMD_PKT) += smd_pkt.o
obj-$(CONFIG_MSM_SMD_NMEA) += smd_nmea.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_DEBUG_FS) += smd_rpc_sym.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_clients.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_xdr.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += rpcrouter_smd_xprt.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += rpc_nv.o
obj-$(CONFIG_MSM_RPC_PING) += ping_mdm_rpc_client.o
obj-$(CONFIG_MSM_RPC_PROC_COMM_TEST) += proc_comm_test.o
obj-$(CONFIG_MSM_RPC_PING) += ping_mdm_rpc_client.o ping_apps_server.o
obj-$(CONFIG_MSM_RPC_OEM_RAPI) += oem_rapi_client.o
obj-$(CONFIG_MSM_RPC_WATCHDOG) += rpc_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVER_WATCHDOG) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVER_TIME_REMOTE) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_ARCH_MSM7X30) += dal_axi.o
obj-$(CONFIG_MSM_RPCSERVER_HANDSET) += rpc_server_handset.o
obj-$(CONFIG_MSM7KV2_AUDIO) += qdsp5v2/
obj-$(CONFIG_PM) += pm-boot.o
obj-$(CONFIG_MSM_IDLE_STATS) += pm-stats.o
obj-$(CONFIG_MSM_PM2) += pm2.o
obj-$(CONFIG_MSM_SPM_V1) += spm.o
obj-$(CONFIG_ARCH_MSM7X30) += acpuclock-7x30.o clock-7x30.o clock-local.o clock-pll.o
obj-$(CONFIG_ARCH_MSM7X30) += gpiomux-v1.o gpiomux.o

obj-$(CONFIG_MACH_HUAWEI) += huawei/
obj-$(CONFIG_MACH_SEMC) += semc/

obj-$(CONFIG_MSM_RMT_STORAGE_CLIENT) += rmt_storage_client.o
obj-$(CONFIG_MSM_SHOW_RESUME_IRQ) += msm_show_resume_irq.o
obj-$(CONFIG_MSM_MODEM_RESTART) += restart_7k.o

obj-$(CONFIG_MSM_PROC_COMM_REGULATOR) += proccomm-regulator.o
obj-$(CONFIG_MSM_RPC_PMIC) += pmic.o
obj-$(CONFIG_MSM_RPC_USB) += rpc_hsusb.o rpc_fsusb.o
obj-$(CONFIG_MSM_RPC_PMAPP) += rpc_pmapp.o
