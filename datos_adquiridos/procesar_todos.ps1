$script = "convertir_a_csv.py"

Get-ChildItem -Filter *.txt | ForEach-Object {
    $salida = [System.IO.Path]::ChangeExtension($_.Name, ".csv")
    Write-Host "Procesando: $($_.Name) -> $salida"
    python $script $_.Name $salida
}

Write-Host "Listo."