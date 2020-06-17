<?php
	//echo "POST: completo";
	$infoLuz = "<p>Indice de iluminacao (0-1023): " . $_POST["luz"] . "</p>";
	$infoUmidade = "<p>Umidade: " . $_POST["umidade"] . "%</p>";
	$infoTemp = "<p>Temperatura: ". $_POST["temp"] . " 'C\n<p/>";
	$tituloAcel = "<p>Valores de acelerometro</p>";
	$infoAcelX = "<p> Eixo x: " . $_POST["acelX"] . "</p>";
	$infoAcelY = "<p> Eixo y: " . $_POST["acelY"] . "</p>";
	$infoAcelZ = "<p> Eixo z: " . $_POST["acelZ"] . "</p>";
	$numeroPedestres = "<p> Pedestres ate o momento: " . $_POST["pedestres"] . "</p>";
	
	$resumo = $infoLuz . $infoUmidade . $infoTemp . $tituloAcel . $infoAcelX . $infoAcelY . $infoAcelZ . $numeroPedestres; //string que compoe todos os valores
	file_put_contents("resumo.html", $resumo); //cria uma página HTML chamada resumo e atribui a variavel $resumo como valor padrão
?>